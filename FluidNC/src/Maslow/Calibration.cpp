#include "Calibration.h"
#include "CalibrationSampleWindow.h"
#include "SavedAnchorRecovery.h"
#include "Maslow.h"
#include "../Kinematics/MaslowKinematics.h"
#include "../Serial.h"
#include "../Settings.h"
#include "../System.h"
#include "SquareCalculation.h"
#include <esp_task_wdt.h>
#include <algorithm>

// Bootstrap17 (first-anchor) profile tolerance.  The sled creeps a few mm during
// each tighten/measure cycle on the real frame, so the precision limit cannot be
// met until the anchors exist.  Keep the value in one place so the startup log
// and the measurement check cannot drift apart.
static constexpr float BOOTSTRAP17_REPEATABILITY_MM  = 15.0f;
static constexpr float BOOTSTRAP17_RMS_LIMIT_MM      = 10.0f;
static constexpr float BOOTSTRAP17_MAX_RESIDUAL_MM   = 30.0f;
#include <cmath>
#include <new>
#include <cstdio>
#include <cstring>
#include <vector>

namespace {
    inline void serviceCalibrationWatchdogs(bool yieldToScheduler = false) {
        // Keep both Maslow's update-loop watchdog and ESP task watchdog fed.
        Maslow.resetUpdateWatchdog();
        esp_task_wdt_reset();
        if (yieldToScheduler) {
            // During long compute loops, cooperatively yield so the scheduler can
            // run idle tasks on this core and avoid task-starvation resets.
            delay(0);
        }
    }

    struct CalibrationMeasurement {
        double tl;
        double tr;
        double bl;
        double br;
    };

    constexpr double LM_INITIAL_LAMBDA        = 0.001;
    constexpr double LM_LAMBDA_INCREASE       = 10.0;
    constexpr double LM_LAMBDA_DECREASE       = 0.1;
    constexpr int    LM_MAX_ITERATIONS        = 100;
    constexpr int    LM_MAX_REJECTIONS        = 20;
    constexpr double LM_CONVERGENCE_THRESHOLD = 1e-4;

    // Fitness gate thresholds — tune against real-machine logs before tightening
    constexpr double FITNESS_RMS_FAIL_MM              = 5.0;   // average belt error too large
    constexpr double FITNESS_MAX_RES_FAIL_MM          = 15.0;  // single-waypoint outlier

    struct CalibrationFitness {
        double rms;             // sqrt(SSR / 4N) — overall fitness analog, units: mm
        double maxResidual;     // max|r_i| — single worst belt-length error, mm
        double rmsPerAnchor[4]; // {tl, tr, bl, br} per-anchor RMS, mm
        bool   converged;       // true if step-norm < threshold within iteration cap
    };

    void bundleResiduals(const std::vector<CalibrationMeasurement>& measurements, const std::vector<double>& params, std::vector<double>& residuals) {
        const double tlX = params[0], tlY = params[1];
        const double trX = params[2], trY = params[3];
        const double brX = params[4];

        residuals.assign(measurements.size() * 4, 0.0);
        for (size_t i = 0; i < measurements.size(); i++) {
            const double sx = params[5 + 2 * i];
            const double sy = params[5 + 2 * i + 1];
            const auto&  m  = measurements[i];

            const double dTl = std::sqrt((sx - tlX) * (sx - tlX) + (sy - tlY) * (sy - tlY));
            const double dTr = std::sqrt((sx - trX) * (sx - trX) + (sy - trY) * (sy - trY));
            const double dBl = std::sqrt((sx) * (sx) + (sy) * (sy));
            const double dBr = std::sqrt((sx - brX) * (sx - brX) + (sy) * (sy));

            residuals[4 * i + 0] = dTl - m.tl;
            residuals[4 * i + 1] = dTr - m.tr;
            residuals[4 * i + 2] = dBl - m.bl;
            residuals[4 * i + 3] = dBr - m.br;
        }
    }

    double sumSquaredResiduals(const std::vector<double>& residuals) {
        double sum = 0.0;
        for (const double r : residuals) {
            sum += r * r;
        }
        return sum;
    }

    void estimateSledPosition(const CalibrationMeasurement& measurement, double tlX, double tlY, double trX, double trY, double brX, double& sx, double& sy) {
        sx = (tlX + trX) / 2.0;
        sy = (tlY + trY) / 4.0;

        const double anchorX[4] = { tlX, trX, 0.0, brX };
        const double anchorY[4] = { tlY, trY, 0.0, 0.0 };
        const double belt[4]    = { measurement.tl, measurement.tr, measurement.bl, measurement.br };

        for (int iter = 0; iter < 20; iter++) {
            double hxx = 0.0, hyy = 0.0, hxy = 0.0, gx = 0.0, gy = 0.0;
            for (int j = 0; j < 4; j++) {
                const double dx = sx - anchorX[j];
                const double dy = sy - anchorY[j];
                const double d  = std::sqrt(dx * dx + dy * dy) + 1e-10;
                const double r  = d - belt[j];
                const double jx = dx / d;
                const double jy = dy / d;
                hxx += jx * jx;
                hyy += jy * jy;
                hxy += jx * jy;
                gx += r * jx;
                gy += r * jy;
            }

            const double det = hxx * hyy - hxy * hxy + 1e-12;
            sx -= (hyy * gx - hxy * gy) / det;
            sy -= (hxx * gy - hxy * gx) / det;

            if (std::abs(gx) + std::abs(gy) < 1e-8) {
                break;
            }
        }
    }

    void measurementJacobiansAndResiduals(const CalibrationMeasurement& measurement,
                                          double                        tlX,
                                          double                        tlY,
                                          double                        trX,
                                          double                        trY,
                                          double                        brX,
                                          double                        sx,
                                          double                        sy,
                                          double (&jia)[4][5],
                                          double (&jis)[4][2],
                                          double (&ri)[4]) {
        for (size_t row = 0; row < 4; row++) {
            for (size_t col = 0; col < 5; col++) {
                jia[row][col] = 0.0;
            }
            jis[row][0] = 0.0;
            jis[row][1] = 0.0;
        }

        const double dxTl = sx - tlX;
        const double dyTl = sy - tlY;
        const double dTl  = std::sqrt(dxTl * dxTl + dyTl * dyTl) + 1e-12;

        const double dxTr = sx - trX;
        const double dyTr = sy - trY;
        const double dTr  = std::sqrt(dxTr * dxTr + dyTr * dyTr) + 1e-12;

        const double dBl = std::sqrt(sx * sx + sy * sy) + 1e-12;

        const double dxBr = sx - brX;
        const double dBr  = std::sqrt(dxBr * dxBr + sy * sy) + 1e-12;

        ri[0] = dTl - measurement.tl;
        ri[1] = dTr - measurement.tr;
        ri[2] = dBl - measurement.bl;
        ri[3] = dBr - measurement.br;

        jia[0][0] = -dxTl / dTl;
        jia[0][1] = -dyTl / dTl;
        jis[0][0] = dxTl / dTl;
        jis[0][1] = dyTl / dTl;

        jia[1][2] = -dxTr / dTr;
        jia[1][3] = -dyTr / dTr;
        jis[1][0] = dxTr / dTr;
        jis[1][1] = dyTr / dTr;

        jis[2][0] = sx / dBl;
        jis[2][1] = sy / dBl;

        jia[3][4] = -dxBr / dBr;
        jis[3][0] = dxBr / dBr;
        jis[3][1] = sy / dBr;
    }

    bool invertDamped2x2(double v00, double v01, double v11, double lambda, double (&inverse)[2][2]) {
        const double d00 = v00 + lambda * std::max(v00, 1e-10);
        const double d11 = v11 + lambda * std::max(v11, 1e-10);
        const double det = d00 * d11 - v01 * v01;
        if (std::abs(det) < 1e-12) {
            return false;
        }

        const double invDet = 1.0 / det;
        inverse[0][0]       = d11 * invDet;
        inverse[0][1]       = -v01 * invDet;
        inverse[1][0]       = -v01 * invDet;
        inverse[1][1]       = d00 * invDet;
        return true;
    }

    bool solve5x5(double matrix[5][5], const double rhs[5], double solution[5]) {
        double b[5];
        for (size_t i = 0; i < 5; i++) {
            b[i]        = rhs[i];
            solution[i] = 0.0;
        }

        for (size_t col = 0; col < 5; col++) {
            size_t pivot = col;
            for (size_t row = col + 1; row < 5; row++) {
                if (std::abs(matrix[row][col]) > std::abs(matrix[pivot][col])) {
                    pivot = row;
                }
            }
            if (std::abs(matrix[pivot][col]) < 1e-12) {
                return false;
            }
            if (pivot != col) {
                for (size_t k = col; k < 5; k++) {
                    std::swap(matrix[col][k], matrix[pivot][k]);
                }
                std::swap(b[col], b[pivot]);
            }

            const double pivotValue = matrix[col][col];
            for (size_t row = col + 1; row < 5; row++) {
                const double factor = matrix[row][col] / pivotValue;
                matrix[row][col]    = 0.0;
                for (size_t k = col + 1; k < 5; k++) {
                    matrix[row][k] -= factor * matrix[col][k];
                }
                b[row] -= factor * b[col];
            }
        }

        for (int row = 4; row >= 0; row--) {
            double sum = b[row];
            for (size_t col = row + 1; col < 5; col++) {
                sum -= matrix[row][col] * solution[col];
            }
            solution[row] = sum / matrix[row][row];
        }
        return true;
    }

    void accumulatePointBlocks(const CalibrationMeasurement& measurement,
                               double                        tlX,
                               double                        tlY,
                               double                        trX,
                               double                        trY,
                               double                        brX,
                               double                        sx,
                               double                        sy,
                               double (&u)[5][5],
                               double (&ga)[5],
                               double (&w)[5][2],
                               double (&gi)[2],
                               double& v00,
                               double& v01,
                               double& v11) {
        double jia[4][5];
        double jis[4][2];
        double ri[4];
        measurementJacobiansAndResiduals(measurement, tlX, tlY, trX, trY, brX, sx, sy, jia, jis, ri);

        v00 = 0.0;
        v01 = 0.0;
        v11 = 0.0;
        gi[0] = 0.0;
        gi[1] = 0.0;
        for (size_t a = 0; a < 5; a++) {
            w[a][0] = 0.0;
            w[a][1] = 0.0;
        }

        for (size_t row = 0; row < 4; row++) {
            const double js0 = jis[row][0];
            const double js1 = jis[row][1];
            v00 += js0 * js0;
            v01 += js0 * js1;
            v11 += js1 * js1;
            gi[0] += js0 * ri[row];
            gi[1] += js1 * ri[row];

            for (size_t a = 0; a < 5; a++) {
                const double ja = jia[row][a];
                ga[a] += ja * ri[row];
                w[a][0] += ja * js0;
                w[a][1] += ja * js1;
                for (size_t b = a; b < 5; b++) {
                    u[a][b] += ja * jia[row][b];
                }
            }
        }
    }
}  // namespace

// Helper function to get MaslowKinematics instance
static Kinematics::MaslowKinematics* getKinematics() {
    using namespace Kinematics;
    MaslowKinematics* kinematics = getMaslowKinematics();
    if (!kinematics) {
        log_error("MaslowKinematics not available");
    }
    return kinematics;
}

// #define UNKNOWN 0
// #define RETRACTING 1
// #define RETRACTED 2
// #define EXTENDING 3
// #define EXTENDEDOUT 4 //Extended is a reserved word
// #define TAKING_SLACK 5
// #define CALIBRATION_IN_PROGRESS 6
// #define READY_TO_CUT 7
// #define RELEASE_TENSION 8
// #define CALIBRATION_COMPUTING 9

// Constructor
Calibration::Calibration() {
    currentState = UNKNOWN;
    // Initialize calibration loop state variables
    calibrationDirection  = UP;
    measurementInProgress = true;
}

//------------------------------------------------------
//------------------------------------------------------ State Machine Functions
//------------------------------------------------------
int Calibration::getCurrentState() {
    return currentState;
}

//Prints the machine's current state
void Calibration::printCurrentState() {
    log_info("Current state: " << currentState);
}

// Set extended state variables (used when restoring from NVS)
void Calibration::setExtendedState(bool tl, bool tr, bool bl, bool br) {
    extended[_TL] = tl;
    extended[_TR] = tr;
    extended[_BL] = bl;
    extended[_BR] = br;
}

//Request a state change to a new state. Returns true on success and false on failure (although return value is never used atm)
bool Calibration::requestStateChange(int newState) {
    log_info("Requesting state change from " << stateNames[currentState].name << " to " << stateNames[newState].name);

    bool success = false;

    switch (newState) {
        case UNKNOWN:  //We can enter unknown from any stable state (the machine is not currently performing an action)
            currentState = UNKNOWN;
            success      = true;
            break;
        case RETRACTING:  //We can enter retracting from any state
            currentState = RETRACTING;

            retracting[_TL] = true;
            retracting[_TR] = true;
            retracting[_BL] = true;
            retracting[_BR] = true;
            for (int arm = _TL; arm < ARM_COUNT; arm++) {
                Maslow.axis[arm].reset();
            }

            success = true;
            break;
        case RETRACTED:  //We can enter retracted from retracting only
            if (currentState == RETRACTING) {
                currentState = RETRACTED;
                sys.set_state(State::Idle);
                // Explicitly save belt positions now that belts are retracted and tight
                Maslow.saveBeltPositions();
                success = true;
                break;
            } else {
                break;
            }
        case EXTENDING:  //We can enter extending from retracted or extended out
            if (currentState == RETRACTED || currentState == EXTENDEDOUT) {
                currentState = EXTENDING;
                Maslow.stop();
                sys.set_state(State::Homing);

                extended[_TL] = false;
                extended[_TR] = false;
                extended[_BL] = false;
                extended[_BR] = false;

                updateCenterXY();  //Why is this needed here?
                success = true;
                break;
            } else {
                log_info("Cannot extend the belts until they have been retracted");
                break;
            }
        case EXTENDEDOUT:  //We can enter extended from extending or in the event of a failure from taking slack, release tension, or calibration computing
            if (currentState == EXTENDING || currentState == TAKING_SLACK || currentState == RELEASE_TENSION ||
                currentState == CALIBRATION_COMPUTING || currentState == CALIBRATION_IN_PROGRESS) {
                currentState = EXTENDEDOUT;
                sys.set_state(State::Idle);
                // Save belt positions now that belts are extended and at a known position
                Maslow.saveBeltPositions();
                success = true;
                break;
            } else {
                break;
            }
        case TAKING_SLACK:
            if (keepSettledSavedAnchorHold()) {
                success = true;
                break;
            }
            if (currentState == READY_TO_CUT && savedAnchorHoldActive && applyTensionRecovery.motionSettling) {
                log_info("Apply Tension requires the current motion to settle");
                break;
            }
            // Idle is also required when retrying a previously aborted restore.
            if (sys.state() == State::Idle &&
                (currentState == EXTENDEDOUT || currentState == READY_TO_CUT || currentState == TAKING_SLACK) &&
                beginApplyTension()) {
                currentState = TAKING_SLACK;
                sys.set_state(State::Homing);
                takeSlack = true;
                success = true;
            } else {
                log_info("Apply Tension requires Idle, extended belts and a valid belt reference");
            }
            break;
        case CALIBRATION_IN_PROGRESS:  //We can enter calibration in progress from EXTENDEDOUT, READY_TO_CUT, or CALIBRATION_COMPUTING
            if (currentState == EXTENDEDOUT || currentState == READY_TO_CUT || currentState == CALIBRATION_COMPUTING) {
                // If we're coming from CALIBRATION_COMPUTING and calibration is already complete,
                // go directly to READY_TO_CUT instead of cycling through CALIBRATION_IN_PROGRESS
                if (currentState == CALIBRATION_COMPUTING && waypoint > pointCount && pointCount > 0) {
                    log_info("Find Anchors already complete (waypoint " << waypoint << " > pointCount " << pointCount
                             << "), transitioning directly to READY_TO_CUT");
                    resetCalibrationState();
                    return requestStateChange(READY_TO_CUT);
                }

                // When entering from EXTENDEDOUT this is always a fresh calibration start (either
                // the first run or a re-run after a failure).  Reset waypoint counters and fitness
                // tracking so stale data from any previous run cannot affect this run.
                if (currentState == EXTENDEDOUT) {
                    diagnosticPhase = nullptr;
                    diagnosticWaypoint = -1;
                    diagnosticRun = -1;
                    diagnosticTightMask = 0;
                    diagnosticCurrentThreshold = 0;
                    diagnosticPhaseStart = diagnosticLastLog = millis();
                    for (int arm = 0; arm < ARM_COUNT; ++arm)
                        diagnosticEncoderBaseline[arm] = Maslow.axis[arm].getEncoderReadFailures();
                    log_info("CALDIAG v2 enabled: logging only; snapshots 500ms; no diagnostic timeouts");
                    waypoint            = 0;
                    recomputeCountIndex = 0;
                    previousFitnessRms  = -1.0;
                    lastRecomputePassed = false;
                }

                currentState = CALIBRATION_IN_PROGRESS;

                //Reset the axis targets at the beginning of calibration
                Maslow.axis[_TL].setTarget(Maslow.axis[_TL].getPosition());
                Maslow.axis[_TR].setTarget(Maslow.axis[_TR].getPosition());

                sys.set_state(State::Homing);

                //If we are at the first point we need to generate the grid before we can start
                if (waypoint == 0) {
                    resetMeasurementState();
                    // Initialize calibration loop state for fresh start
                    calibrationDirection     = UP;
                    measurementInProgress    = true;
                    orientationDetectionDone = false;  // Reset orientation detection flag for new calibration
                    orientationDetectTimer   = 0;      // Reset timer so detection starts fresh

                    if (!generate_calibration_grid()) {
                        log_error("Find Anchors start failed: invalid calibration grid/range");
                        resetCalibrationState();
                        requestStateChange(EXTENDEDOUT);
                        return false;
                    }
                }
                //We have just finished the first six points and have updated anchor locations so it's time to generate the grid again
                //if the user has selected for it to be generated automatically
                if (waypoint == 6 && calibration_grid_width_mm_X == 0 && calibration_grid_height_mm_Y == 0) {
                    if (!generate_calibration_grid()) {
                        log_error("Find Anchors start failed: invalid calibration grid/range");
                        resetCalibrationState();
                        requestStateChange(EXTENDEDOUT);
                        return false;
                    }
                }
                Maslow.stop();

                //Save the z-axis 'stop' position
                Maslow.targetZ = 0;
                Maslow.setZStop();

                //Recalculate the center position because the machine dimensions may have been updated
                updateCenterXY();

                //At this point it's likely that we have just sent the machine new cordinates for the anchor points so we need to figure out our new XY
                //cordinates by looking at the current lengths of the top two belts.
                //If we can't load the position, that's OK, we can still go ahead with the calibration and the first point will make a guess for it
                float x          = 0;
                float y          = 0;
                auto  kinematics = getKinematics();

                // Get current Z position to accurately convert measurements to XY plane
                float* mpos     = get_mpos();
                float  currentZ = mpos[2];

                // Compute total vertical distances for TL and TR
                float tlTotalZ = (currentZ + kinematics->getTlZ() + kinematics->getSpoilboardThickness() + kinematics->getWorkThickness());
                float trTotalZ = (currentZ + kinematics->getTrZ() + kinematics->getSpoilboardThickness() + kinematics->getWorkThickness());

                if (kinematics && computeXYfromLengths(measurementToXYPlane(Maslow.axis[_TL].getPosition(), tlTotalZ),
                                                       measurementToXYPlane(Maslow.axis[_TR].getPosition(), trTotalZ),
                                                       x,
                                                       y)) {
                    //We reset the last waypoint to where it actually is so that we can move from the updated position to the next waypoint
                    if (waypoint > 0) {
                        calibrationGrid[waypoint - 1][0] = x;
                        calibrationGrid[waypoint - 1][1] = y;
                    }

                    log_info("Machine Position found as X: " << x << " Y: " << y);

                    //Set the internal machine position using actual belt positions to avoid synchronization issues
                    // Get current belt positions from hardware and set motor steps directly
                    float beltLength[ARM_COUNT];
                    for (int arm = _TL; arm < ARM_COUNT; arm++) {
                        beltLength[arm] = Maslow.axis[arm].getPosition();  // Actual belt position from hardware
                    }

                    log_info("Setting motor positions from hardware readings:");
                    log_info("TL: " << beltLength[_TL] << " TR: " << beltLength[_TR] << " BL: " << beltLength[_BL]
                                    << " BR: " << beltLength[_BR]);

                    // Set motor positions directly from hardware readings
                    // Axis mapping: A=TL(0), B=TR(1), C=BL(2), D=BR(3), Z=router(4)
                    set_motor_steps(0, mpos_to_steps(beltLength[_TL], 0));  // A axis = TL belt
                    set_motor_steps(1, mpos_to_steps(beltLength[_TR], 1));  // B axis = TR belt
                    set_motor_steps(2, mpos_to_steps(beltLength[_BL], 2));  // C axis = BL belt
                    set_motor_steps(3, mpos_to_steps(beltLength[_BR], 3));  // D axis = BR belt
                    set_motor_steps(4, mpos_to_steps(0.0, 4));              // Z axis = 0 (surface level) during calibration

                    gc_sync_position();  //This updates the Gcode engine with the new position from the stepping engine that we set with set_motor_steps
                    plan_sync_position();
                }

                sys.set_state(State::Homing);

                calibrationInProgress = true;  //Should be replaced by state machine
                success               = true;
                break;
            } else {
                log_info("Cannot start Find Anchors until the belts have been extended");
                break;
            }
        case CALIBRATION_COMPUTING:  //We can enter calibration computing from calibration in progress
            if (currentState == CALIBRATION_IN_PROGRESS) {
                currentState          = CALIBRATION_COMPUTING;
                calibrationInProgress = false;
                success               = true;
                break;
            } else {
                log_info("Cannot enter Find Anchors computing from state " << stateNames[currentState].name);
                break;
            }
        case READY_TO_CUT:  //We can enter ready to cut from calibration in progress, calibration computing or taking slack
            if (currentState == CALIBRATION_IN_PROGRESS || currentState == CALIBRATION_COMPUTING || currentState == TAKING_SLACK) {
                currentState = READY_TO_CUT;
                
                // Synchronize motor positions with actual belt positions to prevent jogging issues
                // This is especially critical when transitioning from CALIBRATION_COMPUTING
                float beltLength[ARM_COUNT];
                for (int arm = _TL; arm < ARM_COUNT; arm++) {
                    beltLength[arm] = Maslow.axis[arm].getPosition();  // Actual belt position from hardware
                }

                log_info("Synchronizing motor positions for READY_TO_CUT:");
                log_info("TL: " << beltLength[_TL] << " TR: " << beltLength[_TR] << " BL: " << beltLength[_BL]
                                << " BR: " << beltLength[_BR]);

                // Set motor positions directly from hardware readings to ensure motion system accuracy
                // Axis mapping: A=TL(0), B=TR(1), C=BL(2), D=BR(3), Z=router(4)
                set_motor_steps(0, mpos_to_steps(beltLength[_TL], 0));  // A axis = TL belt
                set_motor_steps(1, mpos_to_steps(beltLength[_TR], 1));  // B axis = TR belt
                set_motor_steps(2, mpos_to_steps(beltLength[_BL], 2));  // C axis = BL belt
                set_motor_steps(3, mpos_to_steps(beltLength[_BR], 3));  // D axis = BR belt
                // Z axis position is preserved during transition to READY_TO_CUT

                gc_sync_position();   // Update GCode engine with synchronized position
                plan_sync_position(); // Update motion planner with synchronized position
                
                sys.set_state(State::Idle);
                // Explicitly save belt positions now that calibration/take-slack is complete and belts are tight
                Maslow.saveBeltPositions();
                success = true;
                break;
            } else {
                break;
            }
        case RELEASE_TENSION:  //We can enter release tension from any stable state (the machine is not currently performing an action)
            if (currentState == READY_TO_CUT || currentState == UNKNOWN || currentState == EXTENDEDOUT ||
                currentState == CALIBRATION_COMPUTING || currentState == RELEASE_TENSION) {
                previousState   = (currentState != RELEASE_TENSION) ? currentState : previousState;
                currentState    = RELEASE_TENSION;
                complyCallTimer = millis();
                retracting[_TL] = false;
                retracting[_TR] = false;
                retracting[_BL] = false;
                retracting[_BR] = false;
                for (int arm = _TL; arm < ARM_COUNT; arm++) {
                    Maslow.axis[arm].reset();  //This just resets the thresholds for pull tight
                }
                success = true;
                break;
            } else {
                log_info("Cannot release tension from state " << stateNames[currentState].name);
                break;
            }
        default:
            return false;
    }

    if (success) {
        log_info("Succeeded");
    }

    printCurrentState();

    return success;
}

// Stage names describe the next control operation; snapshots show the last PWM command.
void Calibration::diagnosticStage(const char* stage) {
    if (!calibrationInProgress || currentState != CALIBRATION_IN_PROGRESS) return;
    if (diagnosticPhase && strcmp(diagnosticPhase, stage) == 0) return;
    diagnosticPhase = stage;
    diagnosticPhaseStart = millis();
    diagnosticSnapshot();
}

void Calibration::diagnosticSnapshot() {
    diagnosticLastLog = millis();
    log_info("CALDIAG stage=" << (diagnosticPhase ? diagnosticPhase : "start")
             << " wp=" << waypoint << " run=" << diagnosticRun
             << " elapsed_ms=" << static_cast<uint32_t>(diagnosticLastLog - diagnosticPhaseStart)
             << " tight_mask=" << diagnosticTightMask
             << " threshold_adc=" << diagnosticCurrentThreshold);
    const char* labels[] = { "TL", "TR", "BL", "BR" };
    for (int arm = 0; arm < ARM_COUNT; ++arm) {
        auto& axis = Maslow.axis[arm];
        log_info("CALDIAG " << labels[arm] << " pos=" << axis.getPosition()
                 << " target=" << axis.getTarget() << " pwm=" << axis.getMotorPower()
                 << " current_adc=" << axis.getMotorCurrent()
                 << " enc_fail_run=" << (axis.getEncoderReadFailures() - diagnosticEncoderBaseline[arm]));
    }
}

// Logging has no authority over motor outputs, errors, or calibration state.
void Calibration::diagnosticUpdate() {
    if (!calibrationInProgress || currentState != CALIBRATION_IN_PROGRESS) return;
    if (diagnosticWaypoint != waypoint) {
        diagnosticWaypoint = waypoint;
        diagnosticPhase = nullptr;
        diagnosticPhaseStart = millis();
    }
    if (millis() - diagnosticLastLog >= 500UL) diagnosticSnapshot();
}

// -Maslow homing loop. This is used whenver any of the homing funcitons are active (belts extending or retracting)
void Calibration::home() {
    switch (currentState) {
        case RETRACTING:
            //run all the retract functions untill we hit the current limit
            if (retracting[_TL]) {
                if (Maslow.axis[_TL].retract()) {
                    retracting[_TL] = false;
                    axis_homed[0]   = true;
                    extended[_TL]   = false;
                }
            }
            if (retracting[_TR]) {
                if (Maslow.axis[_TR].retract()) {
                    retracting[_TR] = false;
                    axis_homed[1]   = true;
                    extended[_TR]   = false;
                }
            }
            if (retracting[_BL]) {
                if (Maslow.axis[_BL].retract()) {
                    retracting[_BL] = false;
                    axis_homed[2]   = true;
                    extended[_BL]   = false;
                }
            }
            if (retracting[_BR]) {
                if (Maslow.axis[_BR].retract()) {
                    retracting[_BR] = false;
                    axis_homed[3]   = true;
                    extended[_BR]   = false;
                }
            }

            //Once the limits are hit switch to the next state
            if (!retracting[_TL] && !retracting[_BL] && !retracting[_BR] && !retracting[_TR]) {
                requestStateChange(RETRACTED);
            }

            break;
        case EXTENDING:
            //decompress belts for the first half second
            if (millis() - extendCallTimer < 700) {
                if (millis() - extendCallTimer > 0)
                    Maslow.axis[_BR].decompressBelt();
                if (millis() - extendCallTimer > 150)
                    Maslow.axis[_BL].decompressBelt();
                if (millis() - extendCallTimer > 250)
                    Maslow.axis[_TR].decompressBelt();
                if (millis() - extendCallTimer > 350)
                    Maslow.axis[_TL].decompressBelt();
            }
            //then make all the belts comply until they are extended fully, or user terminates it
            else {
                if (!extended[_TL])
                    extended[_TL] = Maslow.axis[_TL].extend(extendDist);
                if (!extended[_TR])
                    extended[_TR] = Maslow.axis[_TR].extend(extendDist);
                if (!extended[_BL])
                    extended[_BL] = Maslow.axis[_BL].extend(extendDist);
                if (!extended[_BR])
                    extended[_BR] = Maslow.axis[_BR].extend(extendDist);
                if (extended[_TL] && extended[_TR] && extended[_BL] && extended[_BR]) {
                    log_info("All belts extended to " << extendDist << "mm");
                    requestStateChange(EXTENDEDOUT);
                }
            }
            break;
        case TAKING_SLACK:
            if (takeSlackFunc()) {  //Returns true. Requests correct state transition within function
                takeSlack = false;
                deallocateCalibrationMemory();
            }
            break;
        case RELEASE_TENSION:
            {
                unsigned long elapsed = millis() - complyCallTimer;
                //decompress belts for the first 40ms
                if (elapsed < 40) {
                    Maslow.axis[_BR].decompressBelt();
                    Maslow.axis[_BL].decompressBelt();
                    Maslow.axis[_TR].decompressBelt();
                    Maslow.axis[_TL].decompressBelt();
                }
                //comply for 760ms (40-800ms)
                else if (elapsed < 800) {
                    for (int arm = _TL; arm < ARM_COUNT; arm++) {
                        Maslow.axis[arm].comply();
                    }
                }
                //stop motors and wait for settling (800-1300ms)
                else if (elapsed < 1300) {
                    for (int arm = _TL; arm < ARM_COUNT; arm++) {
                        Maslow.axis[arm].stop();
                    }
                }
                //transition to next state after settling period
                else {
                    for (int arm = _TL; arm < ARM_COUNT; arm++) {
                        Maslow.axis[arm].stop();
                    }
                    // Don't call sys.set_state here - it will be called by requestStateChange

                    // If the machine was in READY_TO_CUT, EXTENDEDOUT, or CALIBRATION_COMPUTING state before releasing tension,
                    // return to EXTENDEDOUT state, otherwise go to UNKNOWN
                    if (previousState == READY_TO_CUT || previousState == EXTENDEDOUT || previousState == CALIBRATION_COMPUTING) {
                        requestStateChange(EXTENDEDOUT);
                    } else {
                        requestStateChange(UNKNOWN);
                    }
                }
            }
            break;
        case CALIBRATION_IN_PROGRESS:
            calibration_loop();
            break;
        case CALIBRATION_COMPUTING:
            // In CALIBRATION_COMPUTING state, we're waiting for the computer to acknowledge
            for (int arm = _TL; arm < ARM_COUNT; arm++) {
                Maslow.axis[arm].stop();
            }
            break;
        case READY_TO_CUT:
            // When calibration is complete and we're ready to cut, ensure we're in Idle state
            // and stop any residual motor activity
            sys.set_state(State::Idle);
            for (int arm = _TL; arm < ARM_COUNT; arm++) {
                Maslow.axis[arm].stop();
            }
            break;
    }

    handleMotorOverides();

    //if we are done with all the homing moves, switch system state back to Idle?
    if (currentState != RETRACTING && currentState != EXTENDING && currentState != RELEASE_TENSION &&
        currentState != CALIBRATION_COMPUTING && !calibrationInProgress &&
        !takeSlack && !checkOverides()) {
        sys.set_state(State::Idle);
    }
}

//------------------------------------------------------
//------------------------------------------------------ Homing and calibration functions
//------------------------------------------------------

bool Calibration::recomputeAnchorsWithLevenbergMarquardt(int measurementCount) {
    lastRecomputePassed = false;
    const double rmsLimit = calibrationBootstrap17 ? 10.0 : FITNESS_RMS_FAIL_MM;
    const double maxResidualLimit = calibrationBootstrap17 ? 30.0 : FITNESS_MAX_RES_FAIL_MM;
    try {
        serviceCalibrationWatchdogs(true);

        if (measurementCount < 3 || measurementCount > CALIBRATION_GRID_SIZE_MAX || calibration_data == nullptr) {
            log_error("Find Anchors recompute failed: invalid measurement count/storage");
            return false;
        }

        auto kinematics = getKinematics();
        if (!kinematics) {
            log_error("Find Anchors recompute failed: MaslowKinematics unavailable");
            return false;
        }

        std::vector<CalibrationMeasurement> measurements;
        measurements.reserve(measurementCount);
        for (int i = 0; i < measurementCount; i++) {
            for (int arm = 0; arm < ARM_COUNT; ++arm) {
                if (!std::isfinite(calibration_data[i][arm]) || calibration_data[i][arm] <= 0) {
                    log_error("Find Anchors recompute failed: invalid measurement " << i << " arm=" << arm);
                    return false;
                }
            }
            measurements.push_back({ calibration_data[i][0], calibration_data[i][1], calibration_data[i][2], calibration_data[i][3] });
        }

        // Capture initial anchor estimates for retry perturbations
        const double tlX0 = kinematics->getTlX();
        const double tlY0 = kinematics->getTlY();
        const double trX0 = kinematics->getTrX();
        const double trY0 = kinematics->getTrY();
        const double brX0 = kinematics->getBrX();

        // Perturbation offsets applied to anchor starting positions on each retry.
        // tl and tr are perturbed symmetrically (opposite X) to preserve rough frame symmetry.
        // There are LM_MAX_RETRIES entries — one per retry after the initial attempt (attempt 0).
        // Each retry is cheap (LM converges in tens of iterations) and the watchdog is serviced
        // inside the loop, so a larger retry count does not cause problems on the ESP32.
        constexpr int    LM_MAX_RETRIES     = 10;
        constexpr double LM_PERTURB_SMALL   = 25.0;  // mm — first pass of perturbations
        constexpr double LM_PERTURB_LARGE   = 50.0;  // mm — second pass with larger offsets
        constexpr double LM_LAMBDA_OVERFLOW = 1e12;  // lambda threshold above which LM is considered stalled
        // clang-format off
        const double perturbX[LM_MAX_RETRIES] = {
             LM_PERTURB_SMALL, -LM_PERTURB_SMALL,  0.0,               0.0,
             LM_PERTURB_SMALL, -LM_PERTURB_SMALL,
             LM_PERTURB_LARGE, -LM_PERTURB_LARGE,  0.0,               0.0 };
        const double perturbY[LM_MAX_RETRIES] = {
             0.0,               0.0,               LM_PERTURB_SMALL, -LM_PERTURB_SMALL,
             LM_PERTURB_SMALL, -LM_PERTURB_SMALL,
             0.0,               0.0,               LM_PERTURB_LARGE, -LM_PERTURB_LARGE };
        // clang-format on

        std::vector<double> globalBestParams;
        double              globalBestSSR = std::numeric_limits<double>::infinity();
        bool                anyConverged  = false;

        for (int attempt = 0; attempt <= LM_MAX_RETRIES; attempt++) {
            if (attempt > 0) {
                log_info("Find Anchors LM retry " << attempt << "/" << LM_MAX_RETRIES << " with perturbed anchors");
            }

            const double px = (attempt > 0) ? perturbX[attempt - 1] : 0.0;
            const double py = (attempt > 0) ? perturbY[attempt - 1] : 0.0;

            std::vector<double> params;
            params.reserve(5 + 2 * measurementCount);
            params.push_back(tlX0 + px);
            params.push_back(tlY0 + py);
            params.push_back(trX0 - px);  // symmetric: tr perturbed opposite to tl in X
            params.push_back(trY0 + py);
            params.push_back(brX0);

            for (const auto& measurement : measurements) {
                serviceCalibrationWatchdogs(true);
                double sx = 0.0;
                double sy = 0.0;
                estimateSledPosition(measurement, params[0], params[1], params[2], params[3], params[4], sx, sy);
                params.push_back(sx);
                params.push_back(sy);
            }

            std::vector<double> residuals;
            bundleResiduals(measurements, params, residuals);
            double currentSSR = sumSquaredResiduals(residuals);
            if (!std::isfinite(currentSSR)) {
                log_error("Find Anchors recompute failed: non-finite initial geometry");
                return false;
            }

            std::vector<double> bestParams = params;
            double              bestSSR    = currentSSR;
            double              lambda     = LM_INITIAL_LAMBDA;
            int                 rejections = 0;
            std::vector<double> nextParams;
            std::vector<double> nextResiduals;
            int                 iterationCount = 0;
            bool attemptConverged = false;

        for (int iteration = 0; iteration < LM_MAX_ITERATIONS; iteration++) {
            iterationCount = iteration + 1;
            serviceCalibrationWatchdogs(true);

            const double tlX = params[0], tlY = params[1];
            const double trX = params[2], trY = params[3];
            const double brX = params[4];

            double u[5][5]      = {};
            double ga[5]        = {};
            double schurSub[5][5] = {};
            double schurGain[5] = {};
            double pointGradientMax = 0.0;

            for (size_t i = 0; i < measurements.size(); i++) {
                const size_t sxIndex = 5 + 2 * i;
                const size_t syIndex = sxIndex + 1;

                double w[5][2];
                double gi[2];
                double v00 = 0.0, v01 = 0.0, v11 = 0.0;
                accumulatePointBlocks(
                    measurements[i], tlX, tlY, trX, trY, brX, params[sxIndex], params[syIndex], u, ga, w, gi, v00, v01, v11);
                pointGradientMax = std::max(pointGradientMax, std::max(std::abs(gi[0]), std::abs(gi[1])));

                double invV[2][2];
                if (!invertDamped2x2(v00, v01, v11, lambda, invV)) {
                    log_error("Find Anchors recompute failed: singular 2x2 block at iteration " << iteration);
                    return false;
                }

                for (size_t a = 0; a < 5; a++) {
                    const double winv0 = w[a][0] * invV[0][0] + w[a][1] * invV[1][0];
                    const double winv1 = w[a][0] * invV[0][1] + w[a][1] * invV[1][1];
                    schurGain[a] += winv0 * gi[0] + winv1 * gi[1];
                    for (size_t b = a; b < 5; b++) {
                        schurSub[a][b] += winv0 * w[b][0] + winv1 * w[b][1];
                    }
                }
            }

            double gradientMax = pointGradientMax;
            for (const double g : ga) gradientMax = std::max(gradientMax, std::abs(g) / measurementCount);
            // Exact solutions/numerical optima need not produce a strictly lower SSR.
            // Test the undamped gradient, so a huge lambda cannot fake convergence.
            if (gradientMax < LM_CONVERGENCE_THRESHOLD) {
                attemptConverged = true;
                break;
            }

            for (size_t a = 0; a < 5; a++) {
                for (size_t b = 0; b < a; b++) {
                    u[a][b]        = u[b][a];
                    schurSub[a][b] = schurSub[b][a];
                }
            }

            double schurMatrix[5][5];
            double schurRhs[5];
            for (size_t a = 0; a < 5; a++) {
                for (size_t b = 0; b < 5; b++) {
                    schurMatrix[a][b] = u[a][b] - schurSub[a][b];
                }
                schurMatrix[a][a] += lambda * std::max(u[a][a], 1e-10);
                schurRhs[a] = -ga[a] + schurGain[a];
            }

            double anchorStep[5];
            if (!solve5x5(schurMatrix, schurRhs, anchorStep)) {
                log_error("Find Anchors recompute failed: reduced linear solver did not converge at iteration " << iteration);
                return false;
            }

            nextParams = params;
            for (size_t i = 0; i < 5; i++) {
                nextParams[i] += anchorStep[i];
            }

            for (size_t i = 0; i < measurements.size(); i++) {
                const size_t sxIndex = 5 + 2 * i;
                const size_t syIndex = sxIndex + 1;
                const double sx      = params[sxIndex];
                const double sy      = params[syIndex];

                double jia[4][5];
                double jis[4][2];
                double ri[4];
                measurementJacobiansAndResiduals(measurements[i], tlX, tlY, trX, trY, brX, sx, sy, jia, jis, ri);

                double v00 = 0.0, v01 = 0.0, v11 = 0.0;
                double gi[2] = {};
                double w[5][2] = {};
                for (size_t row = 0; row < 4; row++) {
                    const double js0 = jis[row][0];
                    const double js1 = jis[row][1];
                    v00 += js0 * js0;
                    v01 += js0 * js1;
                    v11 += js1 * js1;
                    gi[0] += js0 * ri[row];
                    gi[1] += js1 * ri[row];
                    for (size_t a = 0; a < 5; a++) {
                        w[a][0] += jia[row][a] * js0;
                        w[a][1] += jia[row][a] * js1;
                    }
                }

                double invV[2][2];
                if (!invertDamped2x2(v00, v01, v11, lambda, invV)) {
                    log_error("Find Anchors recompute failed: singular 2x2 block at iteration " << iteration);
                    return false;
                }

                double pointRhs0 = -gi[0];
                double pointRhs1 = -gi[1];
                for (size_t a = 0; a < 5; a++) {
                    pointRhs0 -= w[a][0] * anchorStep[a];
                    pointRhs1 -= w[a][1] * anchorStep[a];
                }

                const double sxStep = invV[0][0] * pointRhs0 + invV[0][1] * pointRhs1;
                const double syStep = invV[1][0] * pointRhs0 + invV[1][1] * pointRhs1;
                nextParams[sxIndex] += sxStep;
                nextParams[syIndex] += syStep;
            }

            bundleResiduals(measurements, nextParams, nextResiduals);
            const double nextSSR = sumSquaredResiduals(nextResiduals);

            double stepNormSquared = 0.0;
            for (size_t i = 0; i < params.size(); ++i) {
                const double step = nextParams[i] - params[i];
                stepNormSquared += step * step;
            }
            if (std::isfinite(nextSSR) && nextSSR < currentSSR) {
                params = std::move(nextParams);
                residuals = std::move(nextResiduals);
                currentSSR = nextSSR;
                lambda = std::max(lambda * LM_LAMBDA_DECREASE, 1e-12);
                rejections = 0;

                if (currentSSR < bestSSR) {
                    bestSSR = currentSSR;
                    bestParams = params;
                }

                if (std::sqrt(stepNormSquared) < LM_CONVERGENCE_THRESHOLD && lambda <= LM_INITIAL_LAMBDA) {
                    attemptConverged = true;
                    break;
                }
            } else {
                lambda *= LM_LAMBDA_INCREASE;
                rejections++;
                if (rejections > LM_MAX_REJECTIONS || lambda > LM_LAMBDA_OVERFLOW) {
                    break;
                }
            }
            // end of LM loop
            }
            const bool thisConverged = attemptConverged;
            log_debug("Find Anchors LM attempt=" << attempt << " iterations=" << iterationCount
                                                 << " bestSSR=" << bestSSR << " converged=" << thisConverged);

            if (thisConverged || bestSSR < globalBestSSR) {
                globalBestSSR    = bestSSR;
                globalBestParams = bestParams;
            }

            if (thisConverged) {
                anyConverged = true;
                break;
            }
        }  // end retry loop

        serviceCalibrationWatchdogs(true);

        // ── Fitness computation ────────────────────────────────────────────────
        if (globalBestParams.size() != static_cast<size_t>(5 + 2 * measurementCount) || !std::isfinite(globalBestSSR)) {
            log_error("Find Anchors recompute failed: no finite solution");
            return false;
        }
        for (const double p : globalBestParams) {
            if (!std::isfinite(p)) return false;
        }
        if (globalBestParams[1] <= 0 || globalBestParams[3] <= 0 || globalBestParams[4] <= 0 ||
            globalBestParams[2] <= globalBestParams[0]) {
            log_error("Find Anchors recompute failed: invalid anchor ordering");
            return false;
        }
        std::vector<double> finalRes;
        bundleResiduals(measurements, globalBestParams, finalRes);

        CalibrationFitness fit;
        fit.rms       = std::sqrt(globalBestSSR / (4.0 * measurementCount));
        fit.converged = anyConverged;

        fit.maxResidual = 0.0;
        for (const double r : finalRes) {
            const double ar = std::abs(r);
            if (ar > fit.maxResidual) {
                fit.maxResidual = ar;
            }
        }

        for (int j = 0; j < 4; j++) {
            double sumSq = 0.0;
            for (int i = 0; i < measurementCount; i++) {
                const double r = finalRes[4 * i + j];
                sumSq += r * r;
            }
            fit.rmsPerAnchor[j] = std::sqrt(sumSq / measurementCount);
        }

        // ── Fitness gates ──────────────────────────────────────────────────────

        // Gate 1: convergence
        if (!fit.converged) {
            log_error("Find Anchors fit failed: LM did not converge after retries - the math solver stalled; try calibrating again");
            return false;
        }

        // Gate 2: overall RMS
        if (!std::isfinite(fit.rms) || fit.rms > rmsLimit) {
            log_error("Find Anchors fit failed: rms=" << fit.rms << "mm exceeds limit " << rmsLimit << "mm");
            return false;
        }

        // Gate 3: max residual — find worst anchor/measurement index for the log
        if (fit.maxResidual > maxResidualLimit) {
            int    worstI = 0, worstJ = 0;
            double worstVal = 0.0;
            for (int i = 0; i < measurementCount; i++) {
                for (int j = 0; j < 4; j++) {
                    const double ar = std::abs(finalRes[4 * i + j]);
                    if (ar > worstVal) {
                        worstVal = ar;
                        worstI   = i;
                        worstJ   = j;
                    }
                }
            }
            log_error("Find Anchors fit failed: maxResidual=" << fit.maxResidual << "mm at anchor=" << worstJ
                                                              << " measurement=" << worstI << " (limit " << maxResidualLimit << "mm)");
            return false;
        }

        // ── All residual gates passed ─────────────────────────────────────────
        log_info("Find Anchors fit: rms=" << fit.rms << "mm max=" << fit.maxResidual << "mm"
                                          << " perAnchor=[" << fit.rmsPerAnchor[0] << "," << fit.rmsPerAnchor[1] << ","
                                          << fit.rmsPerAnchor[2] << "," << fit.rmsPerAnchor[3] << "]mm"
                                          << " converged=" << fit.converged);

        kinematics->setCalibrationAnchors(globalBestParams[0], globalBestParams[1], globalBestParams[2], globalBestParams[3], globalBestParams[4]);
        if (!std::isfinite(kinematics->getCenterX()) || !std::isfinite(kinematics->getCenterY())) {
            kinematics->setCalibrationAnchors(tlX0, tlY0, trX0, trY0, brX0);
            log_error("Find Anchors fit rejected: non-finite frame center; previous anchors restored");
            return false;
        }
        previousFitnessRms = fit.rms;
        lastRecomputePassed = true;
        // The fitted last sled point is expressed in anchor coordinates. Rebase it
        // to the NEW center before the next move; the previous grid point used the old geometry.
        const int last = measurementCount - 1;
        Maslow.x = globalBestParams[5 + 2 * last] - kinematics->getCenterX();
        Maslow.y = globalBestParams[6 + 2 * last] - kinematics->getCenterY();
        if (calibrationGrid != nullptr) {
            calibrationGrid[last][0] = Maslow.x;
            calibrationGrid[last][1] = Maslow.y;
        }
        log_info("Find Anchors recompute complete: tl=(" << globalBestParams[0] << "," << globalBestParams[1] << ") tr=(" << globalBestParams[2] << ","
                                                        << globalBestParams[3] << ") brX=" << globalBestParams[4]
                                                        << " SSR=" << globalBestSSR << " points=" << measurementCount);
        return true;
    } catch (const std::bad_alloc&) {
        log_error("Find Anchors recompute failed: out of memory at points=" << measurementCount);
        return false;
    } catch (...) {
        log_error("Find Anchors recompute failed: unexpected exception at points=" << measurementCount);
        return false;
    }
}

void Calibration::logClbmMeasurements(int measurementCount) const {
    if (measurementCount <= 0 || calibration_data == nullptr) {
        return;
    }

    std::string clbm = "CLBM:[";
    clbm.reserve(16 + static_cast<size_t>(measurementCount) * 48);

    char item[64];
    for (int i = 0; i < measurementCount; i++) {
        snprintf(item,
                 sizeof(item),
                 "{bl:%g, br:%g, tr:%g, tl:%g}",
                 calibration_data[i][_BL],
                 calibration_data[i][_BR],
                 calibration_data[i][_TR],
                 calibration_data[i][_TL]);
        clbm += item;
        if (i + 1 < measurementCount) {
            clbm += ",";
        }
    }
    clbm += "]";

    log_info(clbm.c_str());
}

bool Calibration::updateExtendDistanceFromAnchors() {
    auto kinematics = getKinematics();
    if (!kinematics) {
        log_error("Find Anchors completed, but MaslowKinematics is unavailable for updating " << M << "_Extend_Dist");
        return false;
    }

    constexpr float safetyMargin = 100.0f;
    const float     extension    = kinematics->getBeltEndExtension() + kinematics->getArmLength();
    const float     xPos         = Maslow.x;
    const float     yPos         = Maslow.y;
    const float     zPos         = get_mpos()[2];

    const float zTotal[ARM_COUNT] = { zPos + kinematics->getTlZ() + kinematics->getSpoilboardThickness() + kinematics->getWorkThickness(),
                                      zPos + kinematics->getTrZ() + kinematics->getSpoilboardThickness() + kinematics->getWorkThickness(),
                                      zPos + kinematics->getBlZ() + kinematics->getSpoilboardThickness() + kinematics->getWorkThickness(),
                                      zPos + kinematics->getBrZ() + kinematics->getSpoilboardThickness() + kinematics->getWorkThickness() };

    const float beltLength[ARM_COUNT] = { kinematics->computeTL(xPos, yPos, zPos),
                                          kinematics->computeTR(xPos, yPos, zPos),
                                          kinematics->computeBL(xPos, yPos, zPos),
                                          kinematics->computeBR(xPos, yPos, zPos) };

    float distanceToAnchor[ARM_COUNT] = {};
    for (int arm = 0; arm < ARM_COUNT; arm++) {
        distanceToAnchor[arm] = measurementToXYPlane(beltLength[arm], fabsf(zTotal[arm]));
        if (!std::isfinite(distanceToAnchor[arm])) {
            log_error("Find Anchors completed, but invalid anchor distance prevented updating " << M << "_Extend_Dist");
            return false;
        }
    }

    const float trBlDiagonalAverage   = 0.5f * (distanceToAnchor[_TR] + distanceToAnchor[_BL]);
    const float tlBrDiagonalAverage   = 0.5f * (distanceToAnchor[_TL] + distanceToAnchor[_BR]);
    const float longestDiagonalAverage = std::max(trBlDiagonalAverage, tlBrDiagonalAverage);

    const float computedExtendDistance = longestDiagonalAverage + safetyMargin - extension;
    if (!std::isfinite(computedExtendDistance)) {
        log_error("Find Anchors completed, but computed " << M << "_Extend_Dist is invalid");
        return false;
    }

    extendDist = std::max(0.0f, computedExtendDistance);
    log_info("Find Anchors set " << M << "_Extend_Dist=" << extendDist << " (TR-BL avg=" << trBlDiagonalAverage
                                 << ", TL-BR avg=" << tlBrDiagonalAverage << ", extension=" << extension << ")");
    return true;
}

// --Maslow calibration loop
void Calibration::calibration_loop() {
    diagnosticUpdate();
    serviceCalibrationWatchdogs(false);

    if (waypoint >
        pointCount) {  //Point count is the total number of points to measure so if waypoint > pointcount then the overall measurement process is complete
        // lastRecomputePassed is set to true only inside recomputeAnchorsWithLevenbergMarquardt()
        // after all fitness gates pass; it is reset to false by resetCalibrationState().
        // Calibration always performs at least one recompute before reaching waypoint > pointCount.
        if (lastRecomputePassed) {
            if (!updateExtendDistanceFromAnchors()) {
                log_error("Find Anchors completed, but failed to update " << M << "_Extend_Dist");
                resetCalibrationState();
                requestStateChange(EXTENDEDOUT);
                return;
            }

            char saveCommand[] = "$CO";
            Error saveResult   = execute_line(saveCommand, allChannels, WebUI::AuthenticationLevel::LEVEL_ADMIN);
            if (saveResult != Error::Ok) {
                log_error("Find Anchors completed, but saving configuration failed: " << static_cast<int>(saveResult));
                resetCalibrationState();
                requestStateChange(EXTENDEDOUT);
                return;
            }

            //Reset all of the calibration variables to the defaults so that calibration can be run again
            resetCalibrationState();
            requestStateChange(READY_TO_CUT);
            log_info("Find Anchors complete");
        } else {
            log_error("Find Anchors completed but final fit failed quality gates; anchors NOT saved");
            resetCalibrationState();
            requestStateChange(EXTENDEDOUT);
        }
        return;
    }

    // Run orientation detection at the start of calibration (waypoint == 0)
    // Continue calling detectOrientation() until it returns true (all phases complete)
    if (waypoint == 0 && !detectOrientation()) {
        return;  // Exit early while detection is in progress
    }

    //Taking measurment once we've reached the point
    if (measurementInProgress) {
        if (take_measurement_avg_with_check(waypoint, calibrationDirection)) {  //Takes a measurement and returns true if it's done
            measurementInProgress = false;

            waypoint++;  //Increment the waypoint counter

            if (waypoint > recomputePoints[recomputeCountIndex]) {  //If we have reached the end of this stage of the calibration process
                logClbmMeasurements(waypoint);
                if (!recomputeAnchorsWithLevenbergMarquardt(waypoint)) {
                    log_error("Find Anchors recompute failed");
                    resetCalibrationState();
                    requestStateChange(EXTENDEDOUT);
                    return;
                }
                recomputeCountIndex++;
            } else {
                hold(150);  // Reduced from 250ms to 150ms for faster calibration
            }
        }
    }

    //Move to the next point in the grid
    else {
        if (move_with_slack(calibrationGrid[waypoint - 1][0],
                            calibrationGrid[waypoint - 1][1],
                            calibrationGrid[waypoint][0],
                            calibrationGrid[waypoint][1])) {
            measurementInProgress = true;
            calibrationDirection  = get_direction(
                calibrationGrid[waypoint - 1][0],
                calibrationGrid[waypoint - 1][1],
                calibrationGrid[waypoint][0],
                calibrationGrid[waypoint][1]);  //This is used to set the order that the belts are pulled tight in the following measurement
            Maslow.x = calibrationGrid[waypoint][0];  //Are these ever used anywhere?
            Maslow.y = calibrationGrid[waypoint][1];
            hold(150);  // Reduced from 250ms to 150ms for faster calibration
        }
    }
}

// Restore a pose using saved anchors. No anchor fitting or configuration write occurs here.
bool Calibration::beginApplyTension() {
    const auto kinematics = getKinematics();
    if (!all_axis_homed() || !allAxisExtended() || !kinematics || retractCurrentThreshold <= 0
        || !std::isfinite(applyTensionMaxResidualMm)
        || applyTensionMaxResidualMm < 0.5f || applyTensionMaxResidualMm > 5.0f) return false;

    const uint32_t now = static_cast<uint32_t>(millis());
    ApplyTensionRecoveryState next;
    next.active = true;
    next.started = next.lastDiagnostic = next.lastPreloadUpdate = now;
    for (int arm = _TL; arm < ARM_COUNT; ++arm) {
        auto& axis = Maslow.axis[arm];
        const double length = axis.getPosition();
        if (!axis.hasFreshEncoderFeedback(now) || !std::isfinite(length) || length <= 0) return false;
        next.startLength[arm] = length;
        next.readFailures[arm] = axis.getEncoderReadFailures();
        next.largeSteps[arm] = axis.getEncoderLargeSteps();
    }
    Maslow.stopMotors();
    savedAnchorHoldActive = false;
    applyTensionRecovery = next;
    for (int arm = _TL; arm < ARM_COUNT; ++arm) {
        Maslow.axis[arm].reset();
        Maslow.axis[arm].resetPID();
    }
    log_info("RESTORE profile=saved_anchors_jog_fit_r1 phase=preload center_frame_x=" << kinematics->getCenterX()
             << " center_frame_y=" << kinematics->getCenterY()
             << " sample_span_limit=" << MaslowSavedAnchorRecovery::repeatabilityLimitMm
             << "mm geometry_limit=" << applyTensionMaxResidualMm << "mm");
    return true;
}

bool Calibration::failApplyTension(const char* reason, bool referenceInvalid) {
    const bool readyHold = currentState == READY_TO_CUT;
    Maslow.stopMotors();
    if (applyTensionRecovery.motionSettling) logSavedAnchorSettling("stopped", static_cast<uint32_t>(millis()));
    if (applyTensionRecovery.sampling && !readyHold) logSavedAnchorSampling("stopped");
    savedAnchorHoldActive = false;
    applyTensionRecovery = ApplyTensionRecoveryState{};
    takeSlack = false;
    if (referenceInvalid) {
        for (int arm = _TL; arm < ARM_COUNT; ++arm) axis_homed[arm] = false;
    }
    // Do not persist failed measurements as a valid position reference.
    // A quality failure alone does not invalidate a continuous encoder zero.
    currentState = referenceInvalid ? UNKNOWN : EXTENDEDOUT;
    sys.set_state(State::Alarm);
    if (readyHold) {
        Maslow.markBeltPositionsStale();
        log_error("Ready hold stopped: " << reason << "; belt_reference_valid=" << !referenceInvalid
                  << " error_mm TL=" << Maslow.axis[_TL].getPositionError() << " TR=" << Maslow.axis[_TR].getPositionError()
                  << " BL=" << Maslow.axis[_BL].getPositionError() << " BR=" << Maslow.axis[_BR].getPositionError());
    } else {
        log_error("Apply Tension stopped: " << reason << "; belt_reference_valid=" << !referenceInvalid);
    }
    printCurrentState();
    return true;
}

bool Calibration::applyTensionFeedbackHealthy(uint32_t now) {
    for (int arm = _TL; arm < ARM_COUNT; ++arm) {
        auto& axis = Maslow.axis[arm];
        if (!axis.hasFreshEncoderFeedback(now) || !std::isfinite(axis.getPosition())
            || axis.getEncoderReadFailures() != applyTensionRecovery.readFailures[arm]
            || axis.getEncoderLargeSteps() != applyTensionRecovery.largeSteps[arm]) return false;
    }
    return true;
}

bool Calibration::updateApplyTensionHold(uint32_t now) {
    auto& recovery = applyTensionRecovery;
    bool conflict = false;
    for (int arm = _TL; arm < ARM_COUNT; ++arm) {
        auto& axis = Maslow.axis[arm];
        if (!axis.recomputeCalibrationHold()) {
            failApplyTension("hold feedback unavailable", true);
            return false;
        }
        conflict = conflict ||
            (std::fabs(axis.getPosition() - axis.getTarget()) > 5.0
             && std::fabs(axis.getMotorPower()) >= MaslowCalibrationHold::maxDrivePWM - 1.0);
    }
    if (!conflict) recovery.holdConflict = false;
    else if (!recovery.holdConflict) {
        recovery.holdConflict = true;
        recovery.holdConflictStarted = now;
    } else if (now - recovery.holdConflictStarted >= 1500) {
        failApplyTension("position hold conflict at output limit", false);
        return false;
    }
    return true;
}

bool Calibration::savedAnchorHoldSettled() {
    for (int arm = _TL; arm < ARM_COUNT; ++arm) {
        auto& axis = Maslow.axis[arm];
        if (!(std::fabs(axis.getPositionError()) <= MaslowCalibrationStability::maxPositionErrorMm)) return false;
    }
    return true;
}

bool Calibration::keepSettledSavedAnchorHold() {
    if (sys.state() != State::Idle || currentState != READY_TO_CUT || !savedAnchorHoldActive ||
        applyTensionRecovery.motionSettling || applyTensionRecovery.holdConflict ||
        !all_axis_homed() || !allAxisExtended() ||
        !applyTensionFeedbackHealthy(static_cast<uint32_t>(millis())) || !savedAnchorHoldSettled()) return false;

    for (int arm = _TL; arm < ARM_COUNT; ++arm) {
        if (!(std::fabs(Maslow.axis[arm].getCalibrationHoldVelocity()) <= 1.0)) return false;
    }
    log_info("Apply Tension complete using existing stable Ready hold");
    return true;
}

void Calibration::logSavedAnchorSettling(const char* result, uint32_t now) const {
    const auto& recovery = applyTensionRecovery;
    log_info("MOTION_SETTLE result=" << result
             << " elapsed_ms=" << static_cast<unsigned>(now - recovery.motionSettlingStarted)
             << " longest_in_tolerance_ms=" << static_cast<unsigned>(recovery.longestMotionWithinToleranceMs)
             << " required_ms=100 limit_mm=" << MaslowCalibrationStability::maxPositionErrorMm
             << " outside_mask=" << recovery.motionOutsideToleranceMask
             << " peak_error_mm TL=" << recovery.motionPeakError[_TL] << " TR=" << recovery.motionPeakError[_TR]
             << " BL=" << recovery.motionPeakError[_BL] << " BR=" << recovery.motionPeakError[_BR]);
}

void Calibration::logSavedAnchorSampling(const char* result) const {
    const auto& recovery = applyTensionRecovery;
    log_info("RESTORE_SAMPLE result=" << result << " snapshots=" << recovery.snapshots
             << " position_error_mask=" << recovery.samplePositionMask << " pwm_limit_mask=" << recovery.samplePwmMask
             << " span_reset_mask=" << recovery.sampleDiagnostics.spanResetMask
             << " gap_resets=" << recovery.sampleDiagnostics.observationGapResets
             << " masks=TL1,TR2,BL4,BR8");
}

bool Calibration::beginSavedAnchorMotionSettling() {
    if (!savedAnchorHoldActive || currentState != READY_TO_CUT) return false;

    auto& recovery = applyTensionRecovery;
    recovery.holdConflict = false;
    recovery.motionSettling = true;
    recovery.motionWithinTolerance = false;
    recovery.longestMotionWithinToleranceMs = 0;
    recovery.motionOutsideToleranceMask = 0;
    std::fill_n(recovery.motionPeakError, ARM_COUNT, 0.0);
    recovery.motionSettlingStarted = static_cast<uint32_t>(millis());
    return true;
}

bool Calibration::maintainSavedAnchorHold() {
    if (!savedAnchorHoldActive || currentState != READY_TO_CUT) return false;
    const uint32_t now = static_cast<uint32_t>(millis());
    if (!applyTensionFeedbackHealthy(now)) return failApplyTension("encoder feedback lost while holding Ready position", true);

    auto& recovery = applyTensionRecovery;
    if (recovery.motionSettling) {
        const bool withinTolerance = savedAnchorHoldSettled();
        if (withinTolerance && !recovery.motionWithinTolerance) recovery.motionWithinToleranceSince = now;
        recovery.motionWithinTolerance = withinTolerance;
        for (int arm = _TL; arm < ARM_COUNT; ++arm) {
            const double error = Maslow.axis[arm].getPositionError();
            if (std::fabs(error) > std::fabs(recovery.motionPeakError[arm])) recovery.motionPeakError[arm] = error;
            if (!(std::fabs(error) <= MaslowCalibrationStability::maxPositionErrorMm))
                recovery.motionOutsideToleranceMask |= 1u << arm;
        }
        if (withinTolerance) {
            recovery.longestMotionWithinToleranceMs = std::max(recovery.longestMotionWithinToleranceMs,
                                                               now - recovery.motionWithinToleranceSince);
        }

        if (withinTolerance && now - recovery.motionWithinToleranceSince >= 100) {
            recovery.motionSettling = false;
            Maslow.saveZPos();
            Maslow.saveBeltPositions();
            logSavedAnchorSettling("settled", now);
        } else if (now - recovery.motionSettlingStarted >= 1500) {
            return failApplyTension("motion endpoint did not settle", false);
        } else {
            // Keep tracking the final planned belt targets with the motion
            // controller until they can be handed off to the lower-duty hold.
            Maslow.recomputePID();
            return true;
        }
    }

    updateApplyTensionHold(now);
    return true;
}

bool Calibration::takeSlackFunc() {
    auto& recovery = applyTensionRecovery;
    const uint32_t now = static_cast<uint32_t>(millis());
    if (!recovery.active) return failApplyTension("no active recovery", false);
    if (!applyTensionFeedbackHealthy(now)) return failApplyTension("stale, failed or discontinuous encoder feedback", true);
    if (now - recovery.started >= 60000) return failApplyTension("60-second recovery limit", false);

    float raw[ARM_COUNT];
    for (int arm = _TL; arm < ARM_COUNT; ++arm) {
        raw[arm] = Maslow.axis[arm].getPosition();
        if (applyTensionAllowLimiting && recovery.startLength[arm] - raw[arm] > applyTensionBeltRetractionLimitMm) {
            const double amount = recovery.startLength[arm] - raw[arm];
            const bool stopped = failApplyTension("belt retraction limit exceeded", false);
            log_warn("RESTORE retraction arm=" << arm << " retracted_mm=" << amount
                     << " limit_mm=" << applyTensionBeltRetractionLimitMm);
            return stopped;
        }
    }

    if (!recovery.sampling && now - recovery.started >= 30000)
        return failApplyTension("preload exceeded 30 seconds", false);
    if (recovery.sampling && now - recovery.samplingStarted >= 12000)
        return failApplyTension("no repeatable held samples within 12 seconds", false);
    if (now - recovery.lastDiagnostic >= 1000) {
        recovery.lastDiagnostic = now;
        log_info("RESTORE phase=" << (recovery.sampling ? "sample" : "preload")
                 << " elapsed_ms=" << now - recovery.started << " snapshots=" << recovery.snapshots
                 << " TL=" << raw[_TL] << " TR=" << raw[_TR] << " BL=" << raw[_BL] << " BR=" << raw[_BR]);
        if (!recovery.sampling) {
            log_info("RESTORE preload_adc TL=" << Maslow.axis[_TL].getMotorCurrent() << " TR=" << Maslow.axis[_TR].getMotorCurrent()
                     << " BL=" << Maslow.axis[_BL].getMotorCurrent() << " BR=" << Maslow.axis[_BR].getMotorCurrent()
                     << " inward_pwm TL=" << -Maslow.axis[_TL].getMotorPower() << " TR=" << -Maslow.axis[_TR].getMotorPower()
                     << " BL=" << -Maslow.axis[_BL].getMotorPower() << " BR=" << -Maslow.axis[_BR].getMotorPower());
        }
    }
    if (!recovery.sampling) {
        const uint32_t elapsed = now - recovery.lastPreloadUpdate;
        if (elapsed < 10) return false;
        recovery.lastPreloadUpdate = now;
        const double seconds = std::min(elapsed, uint32_t(50)) / 1000.0;
        const double threshold = retractCurrentThreshold;

        double current[ARM_COUNT], speed[ARM_COUNT];
        bool allLoaded = true, anyLoaded = false, allResponding = true;
        for (int arm = _TL; arm < ARM_COUNT; ++arm) {
            auto& axis = Maslow.axis[arm];
            current[arm] = axis.getMotorCurrent();
            speed[arm] = axis.getBeltSpeed();
            if (!std::isfinite(current[arm]) || current[arm] < 0 || !std::isfinite(speed[arm]))
                return failApplyTension("invalid preload current or speed", false);

            const bool loaded = recovery.preload[arm].duty > 0
                && MaslowSavedAnchorRecovery::PreloadDrive::loaded(current[arm], speed[arm], threshold);
            recovery.preloadResponding[arm] = recovery.preloadResponding[arm] || loaded || speed[arm] <= -2.0;
            allResponding = allResponding && recovery.preloadResponding[arm];
            allLoaded = allLoaded && loaded;
            anyLoaded = anyLoaded || loaded;
        }

        const double speedLimit = allResponding && !anyLoaded ? MaslowSavedAnchorRecovery::preloadSpeedMmPerSecond
                                                             : MaslowSavedAnchorRecovery::preloadCreepMmPerSecond;
        if (!recovery.stability.update(now, raw, allLoaded)) {
            for (int arm = _TL; arm < ARM_COUNT; ++arm) {
                Maslow.axis[arm].driveIn(recovery.preload[arm].update(seconds, current[arm], speed[arm], threshold, speedLimit));
            }
            return false;
        }

        // All four belts have carried load at the same stable sled pose.
        // Capture one encoder batch, then validate a second stable window under position hold.
        for (int arm = _TL; arm < ARM_COUNT; ++arm) {
            auto& axis = Maslow.axis[arm];
            axis.setTarget(raw[arm]);
            axis.resetPID();
        }
        recovery.stability.reset();
        recovery.sampling = true;
        recovery.samplingStarted = now;
        log_info("RESTORE phase=sample all four loaded references captured together"
                 << " current_adc TL=" << current[_TL] << " TR=" << current[_TR]
                 << " BL=" << current[_BL] << " BR=" << current[_BR]);
        return false;
    }

    if (!updateApplyTensionHold(now)) return true;
    bool held = true;
    for (int arm = _TL; arm < ARM_COUNT; ++arm) {
        auto& axis = Maslow.axis[arm];
        const bool positionOk = std::fabs(raw[arm] - axis.getTarget()) <= MaslowCalibrationStability::maxPositionErrorMm;
        const bool pwmOk = std::fabs(axis.getMotorPower()) < MaslowCalibrationHold::maxDrivePWM - 1.0;
        if (!positionOk) recovery.samplePositionMask |= 1u << arm;
        if (!pwmOk) recovery.samplePwmMask |= 1u << arm;
        held = held && positionOk && pwmOk;
    }
    if (!recovery.stability.update(now, raw, held, &recovery.sampleDiagnostics)) return false;
    recovery.stability.reset();
    if (recovery.snapshots >= 2) {
        for (int arm = _TL; arm < ARM_COUNT; ++arm) recovery.retained[recovery.snapshots - 2][arm] = raw[arm];
    }
    if (++recovery.snapshots < 6) return false;
    const auto repeatability = MaslowCalibrationSamples::evaluate(recovery.retained);
    if (repeatability.span > MaslowSavedAnchorRecovery::repeatabilityLimitMm) {
        if (++recovery.replacements > 4) return failApplyTension("held samples exceeded 1 mm repeatability limit", false);
        MaslowCalibrationSamples::slide(recovery.retained);
        recovery.snapshots = 5;
        log_info("RESTORE held sample rejected span_mm=" << repeatability.span);
        return false;
    }

    auto kinematics = getKinematics();
    if (!kinematics) return failApplyTension("saved geometry unavailable", false);
    const float z = get_mpos()[Z_AXIS];
    // The production forward model handles material thickness and fixedZ once.
    const auto predict = [kinematics, z](int arm, double x, double y) {
        return kinematics->compute(arm, x * Maslow.scaleX, y * Maslow.scaleY, z);
    };
    double mean[ARM_COUNT] = {};
    for (int arm = _TL; arm < ARM_COUNT; ++arm) {
        for (int sample = 0; sample < 4; ++sample) mean[arm] += recovery.retained[sample][arm] * 0.25;
    }
    const auto fit = MaslowSavedAnchorRecovery::fitPosition(mean, predict, []() { serviceCalibrationWatchdogs(false); });
    double latestMax = 0;
    if (fit.converged) {
        for (int arm = _TL; arm < ARM_COUNT; ++arm)
            latestMax = std::max(latestMax, std::fabs(raw[arm] - predict(arm, fit.x, fit.y)));
    }
    log_info("RESTORE fit converged=" << fit.converged << " x=" << fit.x << " y=" << fit.y
             << " rms_mm=" << fit.rms << " max_mm=" << fit.maxResidual << " latest_max_mm=" << latestMax
             << " repeatability_mm=" << repeatability.span << " limit_mm=" << applyTensionMaxResidualMm);
    if (!fit.converged) return failApplyTension("four-belt position fit did not converge", false);
    if (!fit.acceptable(applyTensionMaxResidualMm) || latestMax >= applyTensionMaxResidualMm)
        return failApplyTension("four-belt geometry residual exceeded recovery limit", false);
    if (std::fabs(fit.x - Maslow.workAreaCenterOffsetX) > Maslow.workAreaX * 0.5
        || std::fabs(fit.y - Maslow.workAreaCenterOffsetY) > Maslow.workAreaY * 0.5)
        return failApplyTension("recovered position is outside the configured work area", false);

    // Synchronize the planner with the settled encoder readings on entry to Ready.
    for (int arm = _TL; arm < ARM_COUNT; ++arm) Maslow.axis[arm].setTarget(raw[arm]);
    recovery.active = false;
    if (!requestStateChange(READY_TO_CUT)) return failApplyTension("Ready transition rejected", false);
    savedAnchorHoldActive = true;
    serviceCalibrationWatchdogs(false);
    logSavedAnchorSampling("accepted");
    log_info("Apply Tension complete using saved anchors; elapsed_ms=" << static_cast<uint32_t>(millis() - recovery.started));
    return true;
}

/*
*Computes the current xy cordinates of the sled based on the lengths of the upper two belts
*/
bool Calibration::computeXYfromLengths(double TL, double TR, float& x, float& y) {
    auto kinematics = getKinematics();
    if (!kinematics)
        return false;

    if (!std::isfinite(TL) || !std::isfinite(TR) || TL <= 0 || TR <= 0) return false;
    double tlLength = TL;
    double trLength = TR;

    //Find the intersection of the two circles centered at tlX, tlY and trX, trY with radii tlLength and trLength
    double tlX = kinematics->getTlX();
    double tlY = kinematics->getTlY();
    double trX = kinematics->getTrX();
    double trY = kinematics->getTrY();

    double d = sqrt((tlX - trX) * (tlX - trX) + (tlY - trY) * (tlY - trY));
    if (!std::isfinite(d) || d < 1e-6 || d > tlLength + trLength || d < abs(tlLength - trLength)) {
        log_info("Unable to determine machine position");
        return false;
    }

    double a    = (tlLength * tlLength - trLength * trLength + d * d) / (2 * d);
    const double hSquared = tlLength * tlLength - a * a;
    if (hSquared < -1e-6) return false;
    double h    = sqrt(std::max(0.0, hSquared));
    double x0   = tlX + a * (trX - tlX) / d;
    double y0   = tlY + a * (trY - tlY) / d;
    double rawX = x0 + h * (trY - tlY) / d;
    double rawY = y0 - h * (trX - tlX) / d;

    // Adjust to the centered coordinates
    x = rawX - kinematics->getCenterX();
    y = rawY - kinematics->getCenterY();

    return std::isfinite(x) && std::isfinite(y);
}

/**
 * Takes one measurement and returns true when it's done. The result is stored in the passed array.
 * Each measurement is the raw belt length processed into XY plane coordinates.
 * 
 * The function handles two orientations: VERTICAL and HORIZONTAL.
 * 
 * In VERTICAL orientation:
 * - Pulls two bottom belts tight one after another based on the x-coordinate.
 * - Takes a measurement once both belts are tight and stores it in the calibration data array.
 * 
 * In HORIZONTAL orientation:
 * - For the first waypoint (waypoint == 0), pulls all 4 belts tight to ensure proper initial tension
 * - For subsequent waypoints, pulls belts tight based on the direction of the last move.
 * - Takes a measurement once both belts are tight and stores it in the calibration data array.
 * 
 * @param result The array to store the measurement result.
 * @param dir The direction of the last move (UP, DOWN, LEFT, RIGHT). This is used to decide which belts to tighten first
 * @param run The measurement run number at current waypoint (0-3, with first 2 discarded).
 * @param current The current threshold for pulling belts tight.
 * @param waypoint The waypoint number being measured (0 = first waypoint).
 * @return True when the measurement is done, false otherwise.
 */
// One preload per bootstrap waypoint. Repeated snapshots keep the same references;
// they do not repeatedly pull all four belts and move the reference position.
void Calibration::resetHorizontalSampling() {
    horizontalSampling = HorizontalSamplingState{};
    for (int arm = _TL; arm < ARM_COUNT; ++arm) horizontalTight[arm] = false;
    horizontalInitialTensionComplete = false;
    horizontalHoldConflictStart = 0;
}

bool Calibration::failHorizontalSampling(const char* reason) {
    Maslow.stopMotors();
    log_error("Horizontal bootstrap measurement stopped: " << reason
              << " wp=" << horizontalSampling.point);
    resetHorizontalSampling();
    // Do not let the normal EXTENDEDOUT failure transition persist a questionable
    // reference. The acquisition wrapper exits to UNKNOWN for this new policy.
    horizontalSamplingReferenceInvalid = true;
    for (int arm = _TL; arm < ARM_COUNT; ++arm) axis_homed[arm] = false;
    horizontalMeasurementFault = true;
    return false;
}

bool Calibration::takeStableHorizontalMeasurement(float result[4], int current, int point) {
    const unsigned long now = millis();
    constexpr unsigned long tensionTimeoutMs = 30000;
    constexpr unsigned long settlingTimeoutMs = 8000;
    if (horizontalSampling.point != point) {
        resetHorizontalSampling();
        horizontalSampling.point = point;
        horizontalSampling.firstBottom = Maslow.x < 0 ? _BL : _BR;
        horizontalSampling.phaseStarted = now;
        for (int arm = _TL; arm < ARM_COUNT; ++arm) {
            Maslow.axis[arm].stop();
            Maslow.axis[arm].reset();
            Maslow.axis[arm].resetPID();
        }
    }
    for (int arm = _TL; arm < ARM_COUNT; ++arm) {
        if (!std::isfinite(Maslow.axis[arm].getPosition()))
            return failHorizontalSampling("non-finite encoder position");
    }

    if (!horizontalInitialTensionComplete) {
        diagnosticStage("horizontal_initial_tighten");
        if (static_cast<unsigned long>(now - horizontalSampling.phaseStarted) >= tensionTimeoutMs)
            return failHorizontalSampling("initial tension exceeded 30 seconds");
        // Preserve the existing preload behavior while other axes finish. This
        // phase runs only once per waypoint, including across acquisition retries.
        for (int arm = _TL; arm < ARM_COUNT; ++arm) {
            if (Maslow.axis[arm].pull_tight(current)) horizontalTight[arm] = true;
        }
        diagnosticTightMask = (horizontalTight[_TL] ? 1U : 0U) | (horizontalTight[_TR] ? 2U : 0U)
                           | (horizontalTight[_BL] ? 4U : 0U) | (horizontalTight[_BR] ? 8U : 0U);
        if (diagnosticTightMask != 15U) return false;
        for (int arm = _TL; arm < ARM_COUNT; ++arm) {
            Maslow.axis[arm].stop();
            Maslow.axis[arm].reset();
            Maslow.axis[arm].resetPID();
            horizontalSampling.readFailures[arm] = Maslow.axis[arm].getEncoderReadFailures();
            horizontalSampling.largeSteps[arm] = Maslow.axis[arm].getEncoderLargeSteps();
        }
        Maslow.axis[_TL].setTarget(Maslow.axis[_TL].getPosition());
        Maslow.axis[_TR].setTarget(Maslow.axis[_TR].getPosition());
        horizontalTight[_BL] = horizontalTight[_BR] = false;
        horizontalInitialTensionComplete = true;
        horizontalSampling.phaseStarted = now;
        log_info("CALDIAG hold_policy kp=" << MaslowCalibrationHold::proportionalGain
                 << " kd=" << MaslowCalibrationHold::velocityDamping
                 << " effective_pwm_limit=" << MaslowCalibrationHold::maxDrivePWM
                 << " pwm_slew_per_s=" << MaslowCalibrationHold::maxSlewPWMPerSecond);
        log_info("CALDIAG fixed_reference wp=" << point << " TL=" << Maslow.axis[_TL].getTarget()
                 << " TR=" << Maslow.axis[_TR].getTarget());
        return false;
    }

    // Once reference positions are fixed, a discontinuity cannot be repaired by
    // waiting for the wrong cumulative value to look stable. Stop before PID uses it.
    for (int arm = _TL; arm < ARM_COUNT; ++arm) {
        if (Maslow.axis[arm].getEncoderReadFailures() != horizontalSampling.readFailures[arm]
            || Maslow.axis[arm].getEncoderLargeSteps() != horizontalSampling.largeSteps[arm]) {
            log_error("CALDIAG fixed_reference_feedback_fault wp=" << point << " arm=" << arm);
            return failHorizontalSampling("encoder feedback changed after reference capture");
        }
    }

    if (!horizontalSampling.holdingSamples) {
        diagnosticStage("horizontal_tension_fixed_reference");
        if (static_cast<unsigned long>(now - horizontalSampling.phaseStarted) >= tensionTimeoutMs)
            return failHorizontalSampling("bottom tension exceeded 30 seconds");
        if (!Maslow.axis[_TL].recomputeCalibrationHold() || !Maslow.axis[_TR].recomputeCalibrationHold())
            return failHorizontalSampling("stale or invalid hold feedback");
        // Keep a completed bottom belt at its captured position while the other
        // belt tightens. Previously it was left without active position control.
        for (int arm = _BL; arm <= _BR; ++arm) {
            if (horizontalTight[arm] && !Maslow.axis[arm].recomputeCalibrationHold())
                return failHorizontalSampling("stale or invalid hold feedback");
        }
        const int first = horizontalSampling.firstBottom;
        const int second = first == _BL ? _BR : _BL;
        const int pulling = !horizontalTight[first] ? first : second;
        if (!horizontalTight[pulling] && Maslow.axis[pulling].pull_tight(current)) {
            horizontalTight[pulling] = true;
            Maslow.axis[pulling].setTarget(Maslow.axis[pulling].getPosition());
            Maslow.axis[pulling].resetPID();
        }
        diagnosticTightMask = 3U | (horizontalTight[_BL] ? 4U : 0U) | (horizontalTight[_BR] ? 8U : 0U);
        if (horizontalTight[_BL] && horizontalTight[_BR]) {
            horizontalSampling.holdingSamples = true;
            horizontalSampling.phaseStarted = now;
            horizontalSampling.stability.reset();
        }
    } else {
        diagnosticStage("horizontal_settle_fixed_reference");
        for (int arm = _TL; arm < ARM_COUNT; ++arm) {
            if (!Maslow.axis[arm].recomputeCalibrationHold())
                return failHorizontalSampling("stale or invalid hold feedback");
        }
        diagnosticTightMask = 15U;
    }

    bool holdConflict = false;
    for (int arm = _TL; arm < ARM_COUNT; ++arm) {
        if ((arm == _TL || arm == _TR || horizontalTight[arm])
            && std::fabs(Maslow.axis[arm].getPosition() - Maslow.axis[arm].getTarget()) > 25.0
            && std::fabs(Maslow.axis[arm].getMotorPower()) >= MaslowCalibrationHold::maxDrivePWM - 1.0)
            holdConflict = true;
    }
    if (holdConflict) {
        if (horizontalHoldConflictStart == 0) horizontalHoldConflictStart = now;
        else if (static_cast<unsigned long>(now - horizontalHoldConflictStart) >= 1500)
            return failHorizontalSampling("hold target conflict at hold output limit");
    } else {
        horizontalHoldConflictStart = 0;
    }
    if (!horizontalSampling.holdingSamples) return false;
    if (static_cast<unsigned long>(now - horizontalSampling.phaseStarted) >= settlingTimeoutMs)
        return failHorizontalSampling("no stable sampling window within 8 seconds");

    float position[ARM_COUNT];
    bool held = true;
    for (int arm = _TL; arm < ARM_COUNT; ++arm) {
        position[arm] = Maslow.axis[arm].getPosition();
        held = held && std::fabs(position[arm] - Maslow.axis[arm].getTarget())
                           <= MaslowCalibrationStability::maxPositionErrorMm
                    && std::fabs(Maslow.axis[arm].getMotorPower()) < 1000.0;
    }
    if (!horizontalSampling.stability.update(static_cast<uint32_t>(now), position, held)) return false;
    auto kinematics = getKinematics();
    if (!kinematics) return failHorizontalSampling("kinematics unavailable");
    const float z = get_mpos()[2] + kinematics->getSpoilboardThickness() + kinematics->getWorkThickness();
    const float zOffset[ARM_COUNT] = { kinematics->getTlZ(), kinematics->getTrZ(),
                                      kinematics->getBlZ(), kinematics->getBrZ() };
    for (int arm = _TL; arm < ARM_COUNT; ++arm)
        result[arm] = measurementToXYPlane(position[arm], z + zOffset[arm]);
    log_info("CALDIAG measurement wp=" << point << " run=" << diagnosticRun
             << " TL=" << result[_TL] << " TR=" << result[_TR]
             << " BL=" << result[_BL] << " BR=" << result[_BR]);
    // Each of the six acquisitions needs its own continuously observed window.
    // All motor targets and the preload state survive warm-ups and rolling retries.
    horizontalSampling.stability.reset();
    horizontalSampling.phaseStarted = now;
    return true;
}

bool Calibration::take_measurement(float result[4], int dir, int run, int current, int waypoint) {
    serviceCalibrationWatchdogs(false);
    diagnosticRun = run;
    diagnosticCurrentThreshold = current;
    // Apply Tension and vertical/grid measurements retain their existing policy.
    if (orientation == HORIZONTAL && calibrationInProgress && waypoint <= 5) {
        return takeStableHorizontalMeasurement(result, current, waypoint);
    }
    if (orientation == VERTICAL || waypoint > 5) diagnosticStage("measure");

    //Shouldn't this be handled with the same code as below but with the direction set to UP?
    if (orientation == VERTICAL) {
        //first we pull two bottom belts tight one after another, if x<0 we pull left belt first, if x>0 we pull right belt first
        bool& BL_tight = measurementState.bottomLeftTight;
        bool& BR_tight = measurementState.bottomRightTight;
        Maslow.axis[_TL].recomputePID();
        Maslow.axis[_TR].recomputePID();

        // For the first six measurement points (waypoints 0-5), pull belts sequentially
        // After that, pull belts simultaneously for speed
        if (waypoint <= 5) {
            //On the left side of the sheet we want to pull the left belt tight first
            if (Maslow.x < 0) {
                if (!BL_tight) {
                    if (Maslow.axis[_BL].pull_tight(current)) {
                        BL_tight = true;
                        //log_info("Pulled BL tight");
                    }
                    return false;
                }
                if (!BR_tight) {
                    if (Maslow.axis[_BR].pull_tight(current)) {
                        BR_tight = true;
                        //log_info("Pulled BR tight");
                    }
                    return false;
                }
            }

            //On the right side of the sheet we want to pull the right belt tight first
            else {
                if (!BR_tight) {
                    if (Maslow.axis[_BR].pull_tight(current)) {
                        BR_tight = true;
                        //log_info("Pulled BR tight");
                    }
                    return false;
                }
                if (!BL_tight) {
                    if (Maslow.axis[_BL].pull_tight(current)) {
                        BL_tight = true;
                        //log_info("Pulled BL tight");
                    }
                    return false;
                }
            }
        }
        // For subsequent waypoints (6+), pull both belts simultaneously
        else {
            if (Maslow.axis[_BL].pull_tight(current)) {
                BL_tight = true;
            }
            if (Maslow.axis[_BR].pull_tight(current)) {
                BR_tight = true;
            }
        }

        //once both belts are pulled, take a measurement
        if (BR_tight && BL_tight) {
            auto kinematics = getKinematics();
            if (!kinematics)
                return false;

            // Get current Z position to accurately convert measurements to XY plane
            float* mpos     = get_mpos();
            float  currentZ = mpos[2];

            // Compute total vertical distances from each anchor to router (including current Z position)
            float tlTotalZ = (currentZ + kinematics->getTlZ() + kinematics->getSpoilboardThickness() + kinematics->getWorkThickness());
            float trTotalZ = (currentZ + kinematics->getTrZ() + kinematics->getSpoilboardThickness() + kinematics->getWorkThickness());
            float blTotalZ = (currentZ + kinematics->getBlZ() + kinematics->getSpoilboardThickness() + kinematics->getWorkThickness());
            float brTotalZ = (currentZ + kinematics->getBrZ() + kinematics->getSpoilboardThickness() + kinematics->getWorkThickness());

            //take measurement and record it to the calibration data array.
            result[0] = measurementToXYPlane(Maslow.axis[_TL].getPosition(), tlTotalZ);
            result[1] = measurementToXYPlane(Maslow.axis[_TR].getPosition(), trTotalZ);
            result[2] = measurementToXYPlane(Maslow.axis[_BL].getPosition(), blTotalZ);
            result[3] = measurementToXYPlane(Maslow.axis[_BR].getPosition(), brTotalZ);
            BR_tight  = false;
            BL_tight  = false;
            return true;
        }
        return false;
    }
    // in HoRIZONTAL orientation we pull on the belts depending on the direction of the last move. This is important because the other two belts are likely slack
    else if (orientation == HORIZONTAL) {
        // For the first six waypoints (waypoints 0-5), use sequential pulling to handle slack belts properly
        if (waypoint <= 5) {
            bool* tight                     = horizontalTight;
            bool& initial_tension_complete = horizontalInitialTensionComplete;

            // Phase 1: Pull all four belts tight simultaneously to eliminate slack
            if (!initial_tension_complete) {
                diagnosticStage("horizontal_initial_tighten");
                // Keep calling completed axes while the other axes finish. stop()
                // is high impedance on this hardware, so leaving a completed axis
                // idle allows the remaining axes to drag the machine across the frame.
                if (Maslow.axis[_TL].pull_tight(current)) {
                    tight[_TL] = true;
                }
                if (Maslow.axis[_TR].pull_tight(current)) {
                    tight[_TR] = true;
                }
                if (Maslow.axis[_BL].pull_tight(current)) {
                    tight[_BL] = true;
                }
                if (Maslow.axis[_BR].pull_tight(current)) {
                    tight[_BR] = true;
                }

                diagnosticTightMask = (tight[_TL] ? 1U : 0U) | (tight[_TR] ? 2U : 0U)
                                    | (tight[_BL] ? 4U : 0U) | (tight[_BR] ? 8U : 0U);
                // Once all belts are tight, move to phase 2
                if (tight[_TL] && tight[_TR] && tight[_BL] && tight[_BR]) {
                    // The next phase pulls the bottom axes one at a time. Explicitly
                    // clear both outputs so the waiting axis cannot retain a command
                    // from this phase. stop() does not provide active position hold.
                    Maslow.axis[_BL].stop();
                    Maslow.axis[_BR].stop();
                    initial_tension_complete = true;
                    // Set TL and TR targets to their current positions to prevent unwanted movement
                    Maslow.axis[_TL].setTarget(Maslow.axis[_TL].getPosition());
                    Maslow.axis[_TR].setTarget(Maslow.axis[_TR].getPosition());
                    // Reset belt tight flags for the actual measurement phase
                    tight[_BL] = false;
                    tight[_BR] = false;
                }
                return false;
            }

            diagnosticTightMask = (tight[_TL] ? 1U : 0U) | (tight[_TR] ? 2U : 0U)
                                | (tight[_BL] ? 4U : 0U) | (tight[_BR] ? 8U : 0U);
            diagnosticStage("horizontal_measure_bottom");
            // Phase 2: Hold TL and TR in position, then pull BL and BR based on x-coordinate
            // This ensures TL and TR remain as stable reference points for position calculation
            Maslow.axis[_TL].recomputePID();
            Maslow.axis[_TR].recomputePID();

            const bool tlConflict = abs(Maslow.axis[_TL].getTarget() - Maslow.axis[_TL].getPosition()) > 25.0
                                 && abs(Maslow.axis[_TL].getMotorPower()) >= 1000.0;
            const bool trConflict = abs(Maslow.axis[_TR].getTarget() - Maslow.axis[_TR].getPosition()) > 25.0
                                 && abs(Maslow.axis[_TR].getMotorPower()) >= 1000.0;
            if (tlConflict || trConflict) {
                if (horizontalHoldConflictStart == 0) {
                    horizontalHoldConflictStart = millis();
                } else if (millis() - horizontalHoldConflictStart >= 1500) {
                    for (int arm = _TL; arm < ARM_COUNT; ++arm) {
                        Maslow.axis[arm].stop();
                    }
                    log_error("Horizontal measurement stopped: a top hold axis remained over 25mm from target at maximum PWM. "
                              << "TL target=" << Maslow.axis[_TL].getTarget() << " actual=" << Maslow.axis[_TL].getPosition()
                              << " TR target=" << Maslow.axis[_TR].getTarget() << " actual=" << Maslow.axis[_TR].getPosition());
                    for (int arm = _TL; arm < ARM_COUNT; ++arm) horizontalTight[arm] = false;
                    horizontalInitialTensionComplete = false;
                    horizontalHoldConflictStart = 0;
                    horizontalMeasurementFault = true;
                    return false;
                }
            } else {
                horizontalHoldConflictStart = 0;
            }

            // Pull bottom belts based on x-coordinate (same logic as vertical mode)
            if (Maslow.x < 0) {
                // On the left side, pull BL first, then BR
                if (!tight[_BL]) {
                    if (Maslow.axis[_BL].pull_tight(current)) {
                        tight[_BL] = true;
                    }
                    return false;
                }
                if (!tight[_BR]) {
                    if (Maslow.axis[_BR].pull_tight(current)) {
                        tight[_BR] = true;
                    }
                    return false;
                }
            } else {
                // On the right side, pull BR first, then BL
                if (!tight[_BR]) {
                    if (Maslow.axis[_BR].pull_tight(current)) {
                        tight[_BR] = true;
                    }
                    return false;
                }
                if (!tight[_BL]) {
                    if (Maslow.axis[_BL].pull_tight(current)) {
                        tight[_BL] = true;
                    }
                    return false;
                }
            }

            // Once both bottom belts are tight, take the measurement
            if (tight[_BL] && tight[_BR]) {
                auto kinematics = getKinematics();
                if (!kinematics)
                    return false;

                // Get current Z position to accurately convert measurements to XY plane
                float* mpos     = get_mpos();
                float  currentZ = mpos[2];

                // Compute total vertical distances from each anchor to router (including current Z position)
                float tlTotalZ = (currentZ + kinematics->getTlZ() + kinematics->getSpoilboardThickness() + kinematics->getWorkThickness());
                float trTotalZ = (currentZ + kinematics->getTrZ() + kinematics->getSpoilboardThickness() + kinematics->getWorkThickness());
                float blTotalZ = (currentZ + kinematics->getBlZ() + kinematics->getSpoilboardThickness() + kinematics->getWorkThickness());
                float brTotalZ = (currentZ + kinematics->getBrZ() + kinematics->getSpoilboardThickness() + kinematics->getWorkThickness());

                //take measurement and record it to the calibration data array.
                result[0] = measurementToXYPlane(Maslow.axis[_TL].getPosition(), tlTotalZ);
                result[1] = measurementToXYPlane(Maslow.axis[_TR].getPosition(), trTotalZ);
                result[2] = measurementToXYPlane(Maslow.axis[_BL].getPosition(), blTotalZ);
                result[3] = measurementToXYPlane(Maslow.axis[_BR].getPosition(), brTotalZ);
                log_info("CALDIAG measurement wp=" << waypoint << " run=" << run
                         << " TL=" << result[_TL] << " TR=" << result[_TR]
                         << " BL=" << result[_BL] << " BR=" << result[_BR]);
                // Reset all flags for next measurement
                tight[_TL]               = false;
                tight[_TR]               = false;
                tight[_BL]               = false;
                tight[_BR]               = false;
                initial_tension_complete = false;
                return true;
            }
            return false;
        }
        // For subsequent waypoints (6+), pull belts simultaneously for faster measurements
        else {
            MotorUnit* pullAxis1;
            MotorUnit* pullAxis2;
            MotorUnit* holdAxis1;
            MotorUnit* holdAxis2;
            bool& pull1_tight = measurementState.pull1Tight;
            bool& pull2_tight = measurementState.pull2Tight;
            switch (dir) {
                case UP:
                    holdAxis1 = &Maslow.axis[_TL];
                    holdAxis2 = &Maslow.axis[_TR];
                    if (Maslow.x < 0) {
                        pullAxis1 = &Maslow.axis[_BL];
                        pullAxis2 = &Maslow.axis[_BR];
                    } else {
                        pullAxis1 = &Maslow.axis[_BR];
                        pullAxis2 = &Maslow.axis[_BL];
                    }
                    break;
                case DOWN:
                    holdAxis1 = &Maslow.axis[_BL];
                    holdAxis2 = &Maslow.axis[_BR];
                    if (Maslow.x < 0) {
                        pullAxis1 = &Maslow.axis[_TL];
                        pullAxis2 = &Maslow.axis[_TR];
                    } else {
                        pullAxis1 = &Maslow.axis[_TR];
                        pullAxis2 = &Maslow.axis[_TL];
                    }
                    break;
                case LEFT:
                    holdAxis1 = &Maslow.axis[_TL];
                    holdAxis2 = &Maslow.axis[_BL];
                    if (Maslow.y < 0) {
                        pullAxis1 = &Maslow.axis[_BR];
                        pullAxis2 = &Maslow.axis[_TR];
                    } else {
                        pullAxis1 = &Maslow.axis[_TR];
                        pullAxis2 = &Maslow.axis[_BR];
                    }
                    break;
                case RIGHT:
                    holdAxis1 = &Maslow.axis[_TR];
                    holdAxis2 = &Maslow.axis[_BR];
                    if (Maslow.y < 0) {
                        pullAxis1 = &Maslow.axis[_BL];
                        pullAxis2 = &Maslow.axis[_TL];
                    } else {
                        pullAxis1 = &Maslow.axis[_TL];
                        pullAxis2 = &Maslow.axis[_BL];
                    }
                    break;
            }
            holdAxis1->recomputePID();
            holdAxis2->recomputePID();

            if (pullAxis1->pull_tight(current)) {
                pull1_tight = true;
            }
            if (pullAxis2->pull_tight(current)) {
                pull2_tight = true;
            }

            if (pull1_tight && pull2_tight) {
                auto kinematics = getKinematics();
                if (!kinematics)
                    return false;

                // Get current Z position to accurately convert measurements to XY plane
                float* mpos     = get_mpos();
                float  currentZ = mpos[2];

                // Compute total vertical distances from each anchor to router (including current Z position)
                float tlTotalZ = (currentZ + kinematics->getTlZ() + kinematics->getSpoilboardThickness() + kinematics->getWorkThickness());
                float trTotalZ = (currentZ + kinematics->getTrZ() + kinematics->getSpoilboardThickness() + kinematics->getWorkThickness());
                float blTotalZ = (currentZ + kinematics->getBlZ() + kinematics->getSpoilboardThickness() + kinematics->getWorkThickness());
                float brTotalZ = (currentZ + kinematics->getBrZ() + kinematics->getSpoilboardThickness() + kinematics->getWorkThickness());

                //take measurement and record it to the calibration data array.
                result[0]   = measurementToXYPlane(Maslow.axis[_TL].getPosition(), tlTotalZ);
                result[1]   = measurementToXYPlane(Maslow.axis[_TR].getPosition(), trTotalZ);
                result[2]   = measurementToXYPlane(Maslow.axis[_BL].getPosition(), blTotalZ);
                result[3]   = measurementToXYPlane(Maslow.axis[_BR].getPosition(), brTotalZ);
                log_info("CALDIAG measurement wp=" << waypoint << " run=" << run
                         << " TL=" << result[_TL] << " TR=" << result[_TR]
                         << " BL=" << result[_BL] << " BR=" << result[_BR]);
                pull1_tight = false;
                pull2_tight = false;
                return true;
            }
        }
    }

    return false;
}

bool Calibration::take_measurement_avg_with_check(int waypoint, int dir) {
    // Discard two settling samples once, then check a rolling window of four snapshots.
    int& run = measurementState.run;
    int& criticalCounter = measurementState.retries;
    auto& measurements = measurementState.samples;
    float avg = 0, sum = 0;
    const float repeatabilityLimit = calibrationBootstrap17 ? BOOTSTRAP17_REPEATABILITY_MM : 2.5f;
    const int howHardToPull = retractCurrentThreshold;

    const bool measurementTaken = take_measurement(measurements[max(run - 2, 0)], dir, run, howHardToPull, waypoint);
    if (horizontalMeasurementFault) {
        const bool referenceInvalid = horizontalSamplingReferenceInvalid;
        run = 0;
        criticalCounter = 0;
        horizontalMeasurementFault = false;
        resetCalibrationState();
        // UNKNOWN does not save positions. Legacy Apply Tension/other faults keep
        // their existing transition; only fixed-reference sampling invalidates it.
        // A failed direct-motor calibration must leave FluidNC Homing as well.
        // Keep the invalid reference and UNKNOWN state; do not save/re-enable motion.
        if (referenceInvalid) sys.set_state(State::Alarm);
        requestStateChange(referenceInvalid ? UNKNOWN : EXTENDEDOUT);
        return false;
    }
    if (measurementTaken) {  //Throw away measurements are stored in [0]
        for (int arm = 0; arm < ARM_COUNT; ++arm) {
            const float sample = measurements[max(run - 2, 0)][arm];
            if (!std::isfinite(sample) || sample <= 0.0f) {
                log_error("Find Anchors stopped: invalid belt measurement at waypoint " << waypoint << " arm=" << arm);
                resetCalibrationState();
                requestStateChange(EXTENDEDOUT);
                return false;
            }
        }
        if (run < 2) {
            run++;
            return false;  //discard the first two measurements
        }

        run++;

        if (run > 5) {
            run = 0;

            // A whole-window range catches cumulative drift that adjacent differences miss.
            const auto quality = MaslowCalibrationSamples::evaluate(measurements);
            static const char* const armLabels[ARM_COUNT] = { "TL", "TR", "BL", "BR" };
            if (quality.span > repeatabilityLimit) {
                log_error("Measurement error: repeatability span " << quality.span
                          << "mm exceeds " << repeatabilityLimit << "mm; retrying waypoint " << waypoint);
                log_info("CALDIAG repeatability wp=" << waypoint << " axis=" << armLabels[quality.arm]
                         << " span_mm=" << quality.span << " adjacent_mm=" << quality.adjacent
                         << " min_sample=" << quality.minimumSample << " max_sample=" << quality.maximumSample);
                // One compact row per sample, using arm order rather than encoder-port labels.
                for (int sample = 0; sample < 4; ++sample) {
                    log_info("CALDIAG retained sample=" << sample
                             << " TL=" << measurements[sample][_TL] << " TR=" << measurements[sample][_TR]
                             << " BL=" << measurements[sample][_BL] << " BR=" << measurements[sample][_BR]);
                }
                // The previous ten six-acquisition batches allowed 60 acquisitions.
                // Two warm-ups + four initial snapshots + 54 replacements keep that bound.
                if (++criticalCounter >= 55) {
                    log_error("Find Anchors stopped: 60 acquisitions without a stable four-sample window; limit "
                              << repeatabilityLimit << "mm");
                    resetCalibrationState();
                    criticalCounter = 0;
                    requestStateChange(EXTENDEDOUT);
                    return false;
                }
                // Warm-up occurs once per waypoint. Replace only the oldest snapshot, then
                // require four consecutive coherent snapshots before accepting their mean.
                MaslowCalibrationSamples::slide(measurements);
                run = 5;  // The next acquisition writes sample[3] and checks the new window.
                return false;
            }
            log_info("CALDIAG repeatability_ok wp=" << waypoint << " span_mm=" << quality.span
                     << " windows_rejected=" << criticalCounter);

            //If we are measurring the flex we don't want to save the result and instead we want to compare it to the last result
            // COMMENTED OUT: Frame flex measurement calculation disabled
            /*
            if (measureFlex) {
                float newLenTLBR = measurements[0][0] + measurements[0][3];
                float newLenTRBL = measurements[0][1] + measurements[0][2];

                float origLenTLBR = calibration_data[0][0] + calibration_data[0][3];
                float origLenTRBL = calibration_data[0][1] + calibration_data[0][2];

                float diffTLBR = abs(newLenTLBR - origLenTLBR);
                float diffTRBL = abs(newLenTRBL - origLenTRBL);

                log_info("Flex measurement: TLBR: " << diffTLBR << " TRBL: " << diffTRBL);

                measureFlex = false;

                return true;
            }
            */

            //If the measurements seem valid, take the average and record it to the calibration data array. This is the only place we should be writing to the calibration_data array
            for (int i = 0; i < 4; i++) {  //For each axis
                sum                           = measurements[0][i] + measurements[1][i] + measurements[2][i] + measurements[3][i];
                avg                           = sum / 4;
                calibration_data[waypoint][i] = avg;  //This is the only time we should be writing to the calibration data array
                sum                           = 0;
                criticalCounter               = 0;
            }
            log_info("Measured waypoint " << waypoint);
            log_info("Waypoint " << waypoint << " coordinates: X=" << calibrationGrid[waypoint][0] << " Y=" << calibrationGrid[waypoint][1]);

            //A check to see if the results on the first point are within the expected range
            //This logic should only run during calibration, not during Apply Tension
            if (waypoint == 0 && currentState == CALIBRATION_IN_PROGRESS) {
                //Recompute the machine position with the belt lenths and compare the results to that
                float x = 0;
                float y = 0;
                computeXYfromLengths(measurements[0][0], measurements[0][1], x, y);

                //If the frame size is way off, we will compute a rough (assumed to be a square) frame size from the first measurmeent
                auto kinematics = getKinematics();
                if (!kinematics)
                    return false;

                double threshold = 100;
                float  diff[ARM_COUNT];
                diff[_TL] = measurements[0][0] - measurementToXYPlane(kinematics->computeTL(x, y, 0), kinematics->getTlZ());
                diff[_TR] = measurements[0][1] - measurementToXYPlane(kinematics->computeTR(x, y, 0), kinematics->getTrZ());
                diff[_BL] = measurements[0][2] - measurementToXYPlane(kinematics->computeBL(x, y, 0), kinematics->getBlZ());
                diff[_BR] = measurements[0][3] - measurementToXYPlane(kinematics->computeBR(x, y, 0), kinematics->getBrZ());
                log_info("Center point off by: TL: " << diff[_TL] << " TR: " << diff[_TR] << " BL: " << diff[_BL] << " BR: " << diff[_BR]);

                if (abs(diff[_TL]) > threshold || abs(diff[_TR]) > threshold || abs(diff[_BL]) > threshold || abs(diff[_BR]) > threshold) {
                    log_error("Center point off by over " << threshold << "mm");

                    if (!adjustFrameSizeToMatchFirstMeasurement()) {
                        Maslow.eStop("Unable to find a valid frame size to match the first measurement");
                        resetCalibrationState();
                        criticalCounter = 0;
                        requestStateChange(EXTENDEDOUT);
                        return false;
                    }
                }

                //Compute the current XY position from the top two belt measurements...needs to be redone because we've adjusted the frame size by here
                if (!computeXYfromLengths(calibration_data[0][0], calibration_data[0][1], x, y)) {
                    Maslow.eStop("Unable to find machine position from measurements");
                    resetCalibrationState();
                    criticalCounter = 0;
                    requestStateChange(EXTENDEDOUT);
                    return false;
                }

                log_info("Machine Position computed as X: " << x << " Y: " << y);

                //Recompute the first four waypoint locations based on the current position
                calibrationGrid[0][0] =
                    x;  //This first point is never really used because we've already measured here, but it shouldn't be left undefined
                calibrationGrid[0][1] = y;
                calibrationGrid[1][0] = x + coarseStepX;
                calibrationGrid[1][1] = y;
                calibrationGrid[2][0] = x + coarseStepX;
                calibrationGrid[2][1] = y + coarseStepY;
                calibrationGrid[3][0] = x;
                calibrationGrid[3][1] = y + coarseStepY;
                calibrationGrid[4][0] = x - coarseStepX;
                calibrationGrid[4][1] = y + coarseStepY;
                calibrationGrid[5][0] = x - coarseStepX;
                calibrationGrid[5][1] = y;
            }

            //This is the exit to indicate that the measurement was successful

            //Special case where we have a good measurement but we need to take another at this point to measure the flex of the frame
            //Frame flex should only be measured during calibration process, not during "Apply Tension"
            // COMMENTED OUT: Frame flex measurement disabled
            /*
            if (waypoint == 0 && currentState == CALIBRATION_IN_PROGRESS) {
                measureFlex = true;
                log_info("Measuring Frame Flex");
                return false;
            }
            */

            return true;
        }
    }
    //We don't free memory alocated here because we will cycle through again and need it
    return false;
}

// Use slack before the first fit, then follow the selected calibrated move policy.
bool Calibration::move_with_slack(double fromX, double fromY, double toX, double toY) {
    diagnosticStage("move");
    serviceCalibrationWatchdogs(false);

    //This is where we want to introduce some slack so the system
    unsigned long& moveBeginTimer = measurementState.moveBegin;
    bool& decompress = measurementState.moveNeedsInit;
    float                stepSize       = 0.09;  // Increased by 50% from 0.06 for faster calibration movement

    int& direction = measurementState.moveDirection;

    float& xStepSize = measurementState.xStep;
    float& yStepSize = measurementState.yStep;
    bool& tlExtending = measurementState.tlExtending;
    bool& trExtending = measurementState.trExtending;
    bool& blExtending = measurementState.blExtending;
    bool& brExtending = measurementState.brExtending;

    if (!std::isfinite(fromX) || !std::isfinite(fromY) || !std::isfinite(toX) || !std::isfinite(toY)) {
        Maslow.eStop("Invalid calibration movement coordinates");
        return false;
    }
    if (std::abs(toX - fromX) < 1e-6 && std::abs(toY - fromY) < 1e-6) {
        for (int arm = 0; arm < ARM_COUNT; ++arm) {
            Maslow.axis[arm].setTarget(Maslow.axis[arm].getPosition());
            Maslow.axis[arm].stop();
            Maslow.axis[arm].reset();
        }
        decompress = true;
        measurementState.stabilizeTimer = 0;
        return true;
    }

    bool withSlack = true;
    if (waypoint > recomputePoints[0]) {  //If we have completed the first round of calibraiton
        withSlack = false;
    }

    if (calibrationBootstrap17 && orientation == HORIZONTAL && !withSlack) {
        return moveCalibratedHorizontal(fromX, fromY, toX, toY);
    }

    //This runs once at the beginning of the move
    if (decompress) {
        moveBeginTimer = millis();
        decompress     = false;
        direction      = get_direction(fromX, fromY, toX, toY);

        //Compute the X and Y step Size
        if (abs(toX - fromX) > abs(toY - fromY)) {
            xStepSize = (toX - fromX) > 0 ? stepSize : -stepSize;
            yStepSize = ((toY - fromY) > 0 ? stepSize : -stepSize) * abs(toY - fromY) / abs(toX - fromX);
        } else {
            yStepSize = (toY - fromY) > 0 ? stepSize : -stepSize;
            xStepSize = ((toX - fromX) > 0 ? stepSize : -stepSize) * abs(toX - fromX) / abs(toY - fromY);
        }

        //Compute which belts will be getting longer. If the current length is less than the final length the belt needs to get longer
        auto kinematics = getKinematics();
        if (!kinematics)
            return false;

        if (kinematics->computeTL(fromX, fromY, 0) < kinematics->computeTL(toX, toY, 0)) {
            tlExtending = true;
        } else {
            tlExtending = false;
        }
        if (kinematics->computeTR(fromX, fromY, 0) < kinematics->computeTR(toX, toY, 0)) {
            trExtending = true;
        } else {
            trExtending = false;
        }
        if (kinematics->computeBL(fromX, fromY, 0) < kinematics->computeBL(toX, toY, 0)) {
            blExtending = true;
        } else {
            blExtending = false;
        }
        if (kinematics->computeBR(fromX, fromY, 0) < kinematics->computeBR(toX, toY, 0)) {
            brExtending = true;
        } else {
            brExtending = false;
        }

        //Set the target to the starting position
        Maslow.setTargets(fromX, fromY, 0);
    }

    //Decompress belts for 500ms...this happens by returning right away before running any of the rest of the code
    if (millis() - moveBeginTimer < 750 && withSlack) {
        if (orientation == VERTICAL) {
            Maslow.axis[_TL].recomputePID();
            Maslow.axis[_TR].recomputePID();
            Maslow.axis[_BL].decompressBelt();
            Maslow.axis[_BR].decompressBelt();
        } else {
            switch (direction) {
                case UP:
                    Maslow.axis[_BL].decompressBelt();
                    Maslow.axis[_BR].decompressBelt();
                    break;
                case DOWN:
                    Maslow.axis[_TL].decompressBelt();
                    Maslow.axis[_TR].decompressBelt();
                    break;
                case LEFT:
                    Maslow.axis[_TR].decompressBelt();
                    Maslow.axis[_BR].decompressBelt();
                    break;
                case RIGHT:
                    Maslow.axis[_TL].decompressBelt();
                    Maslow.axis[_BL].decompressBelt();
                    break;
            }
        }

        return false;
    }

    //Stop for 50ms
    //we need to stop motors after decompression was finished once
    else if (millis() - moveBeginTimer < 800) {
        Maslow.stopMotors();
        return false;
    }

    //Set the targets
    Maslow.setTargets(Maslow.getTargetX() + xStepSize, Maslow.getTargetY() + yStepSize, 0);

    //Check to see if we have reached our target position
    if (abs(Maslow.getTargetX() - toX) < 5 && abs(Maslow.getTargetY() - toY) < 5) {
        // First, set ALL belt targets to their current position to prevent unwinding
        for (int arm = _TL; arm < ARM_COUNT; arm++) {
            Maslow.axis[arm].setTarget(Maslow.axis[arm].getPosition());
        }

        // Stabilize all belts at their new target positions to prevent unwinding
        for (int arm = _TL; arm < ARM_COUNT; arm++) {
            Maslow.axis[arm].recomputePID();
        }

        // Small delay to allow stabilization
        unsigned long& stabilizeTimer = measurementState.stabilizeTimer;
        if (stabilizeTimer == 0) {
            stabilizeTimer = millis();
            return false;  // Continue stabilizing
        }
        if (millis() - stabilizeTimer < 30) {  // 30ms stabilization period (reduced from 50ms)
            return false;                      // Continue stabilizing
        }
        stabilizeTimer = 0;  // Reset for next waypoint

        finishCalibrationMove(direction);
        return true;
    }

    //In vertical orientation we want to move with the top two belts and always have the lower ones be slack
    if (orientation == VERTICAL) {
        Maslow.axis[_TL].recomputePID();
        Maslow.axis[_TR].recomputePID();
        if (withSlack) {
            Maslow.axis[_BL].comply();
            Maslow.axis[_BR].comply();
        } else {
            Maslow.axis[_BL].recomputePID();
            Maslow.axis[_BR].recomputePID();
        }
    } else {
        //For each belt we check to see if it should be slack
        if (withSlack && tlExtending) {
            Maslow.axis[_TL].comply();
        } else {
            Maslow.axis[_TL].recomputePID();
        }

        if (withSlack && trExtending) {
            Maslow.axis[_TR].comply();
        } else {
            Maslow.axis[_TR].recomputePID();
        }

        if (withSlack && blExtending) {
            Maslow.axis[_BL].comply();
        } else {
            Maslow.axis[_BL].recomputePID();
        }

        if (withSlack && brExtending) {
            Maslow.axis[_BR].comply();
        } else {
            Maslow.axis[_BR].recomputePID();
        }
    }

    return false;  //We have not yet reached our target position
}

// Transfer the same holding pair to the measurement phase in either move mode.
void Calibration::finishCalibrationMove(int direction) {
    int first = _BL;
    int second = _BR;
    if (orientation != VERTICAL) {
        switch (direction) {
            case DOWN:
                first = _TL;
                second = _TR;
                break;
            case LEFT:
                first = _TR;
                second = _BR;
                break;
            case RIGHT:
                first = _TL;
                second = _BL;
                break;
        }
    }
    Maslow.axis[first].stop();
    Maslow.axis[second].stop();
    Maslow.axis[first].reset();
    Maslow.axis[second].reset();
    measurementState.moveNeedsInit = true;
}

// Once the first anchor fit is available, advance all four belts on a shared XY trajectory.
bool Calibration::moveCalibratedHorizontal(double fromX, double fromY, double toX, double toY) {
    constexpr double peakSpeedMmPerSecond = 30.0;
    constexpr double endpointToleranceMm = 2.0;
    constexpr double pauseTrackingErrorMm = 4.0;
    constexpr unsigned long settledMs = 150;
    constexpr unsigned long maxUpdateMs = 20;

    const double dx = toX - fromX;
    const double dy = toY - fromY;
    // The quintic interpolator's peak slope is 1.875 with zero endpoint velocity and acceleration.
    const double durationMs = std::max(500.0, 1875.0 * std::hypot(dx, dy) / peakSpeedMmPerSecond);
    const unsigned long now = millis();
    auto& move = measurementState;
    if (move.moveNeedsInit) {
        if (!getKinematics()) {
            Maslow.eStop("Kinematics unavailable for calibration move");
            return false;
        }
        move.moveNeedsInit = false;
        move.moveBegin = move.moveLastUpdate = now;
        move.moveElapsedMs = 0;
        move.stabilizeTimer = 0;
        Maslow.setTargets(fromX, fromY, 0);
        for (int arm = _TL; arm < ARM_COUNT; ++arm) {
            auto& axis = Maslow.axis[arm];
            // Apply geometric belt-length changes to the measured starting lengths.
            // Carry the offset through this segment so starting tension does not jump.
            move.moveBeltOffset[arm] = axis.getPosition() - axis.getTarget();
            axis.setTarget(axis.getPosition());
            axis.resetPID();
        }
        log_info("CALDIAG move_profile wp=" << waypoint << " duration_ms=" << durationMs
                 << " peak_speed_mm_s=" << peakSpeedMmPerSecond);
    }

    if (now - move.moveBegin >= 3.0 * durationMs + 3000.0) {
        log_error("CALDIAG move_timeout wp=" << waypoint << " profile_elapsed_ms=" << move.moveElapsedMs);
        Maslow.eStop("Calibration move failed to track or settle");
        return false;
    }

    double maxError = 0.0;
    for (int arm = _TL; arm < ARM_COUNT; ++arm) {
        maxError = std::max(maxError, std::fabs(Maslow.axis[arm].getPositionError()));
    }
    // Slow the common trajectory as any belt falls behind. Bound the advance after loop delays.
    const double pace = std::max(0.0, std::min(1.0, (pauseTrackingErrorMm - maxError)
                                                      / (pauseTrackingErrorMm - endpointToleranceMm)));
    const unsigned long deltaMs = std::min(now - move.moveLastUpdate, maxUpdateMs);
    move.moveLastUpdate = now;
    move.moveElapsedMs = std::min(durationMs, move.moveElapsedMs + deltaMs * pace);
    const double t = move.moveElapsedMs / durationMs;
    const double progress = t * t * t * (10.0 + t * (-15.0 + 6.0 * t));
    Maslow.setTargets(fromX + dx * progress, fromY + dy * progress, 0);
    for (int arm = _TL; arm < ARM_COUNT; ++arm) {
        auto& axis = Maslow.axis[arm];
        axis.setTarget(axis.getTarget() + move.moveBeltOffset[arm]);
    }
    Maslow.recomputePID();
    if (!calibrationInProgress) return false;

    // Keep tracking the same final targets until all four belts are in tolerance.
    if (t < 1.0 || maxError > endpointToleranceMm) {
        move.stabilizeTimer = 0;
        return false;
    }
    if (move.stabilizeTimer == 0) move.stabilizeTimer = now;
    if (now - move.stabilizeTimer < settledMs) return false;

    move.stabilizeTimer = 0;
    log_info("CALDIAG move_settled wp=" << waypoint << " max_error_mm=" << maxError
             << " elapsed_ms=" << static_cast<unsigned int>(now - move.moveBegin));
    finishCalibrationMove(get_direction(fromX, fromY, toX, toY));
    return true;
}

//The number of points high and wide  must be an odd number
bool Calibration::generate_calibration_grid() {
    //Allocate memory for the calibration grid
    allocateCalibrationMemory();

    auto kinematics = getKinematics();
    if (!kinematics) return false;
    int gridSize = calibrationBootstrap17 ? 3 : calibrationGridSize;
    const bool automatic = calibration_grid_width_mm_X == 0 || calibration_grid_height_mm_Y == 0;
    float gridWidth = calibration_grid_width_mm_X;
    float gridHeight = calibration_grid_height_mm_Y;
    if (automatic) {
        gridWidth = (kinematics->getTrX() - kinematics->getTlX()) * 0.5f;
        gridHeight = (kinematics->getTlY() - kinematics->getBlY()) * 0.2f;
    }
    if (automatic || calibrationBootstrap17) {
        gridWidth = std::min(gridWidth, Maslow.workAreaX - 200.0f);
        gridHeight = std::min(gridHeight, Maslow.workAreaY - 200.0f);
    }
    if (calibrationBootstrap17) {
        // Keep the existing spacing ceiling even though we now always use 3x3.
        gridWidth = std::min(gridWidth, 2.0f * calibrationMaxSpacingMm);
        gridHeight = std::min(gridHeight, 2.0f * calibrationMaxSpacingMm);
    }
    if (!std::isfinite(gridWidth) || !std::isfinite(gridHeight) || gridWidth <= 0 || gridHeight <= 0) {
        log_error("Invalid calibration range: width=" << gridWidth << " height=" << gridHeight
                  << "; work area must allow the 200mm margin");
        return false;
    }
    if (automatic && !calibrationBootstrap17) {
        const int availableGridSizes[] = {3, 5, 7, 9};
        gridSize = 9;
        for (int size : availableGridSizes) {
            if (std::max(gridWidth, gridHeight) / (size - 1) <= calibrationMaxSpacingMm) {
                gridSize = size;
                break;
            }
        }
    }
    if (gridSize != 3 && gridSize != 5 && gridSize != 7 && gridSize != 9) {
        log_error("Invalid calibration grid size: " << gridSize);
        return false;
    }
    const float xSpacing = gridWidth / (gridSize - 1);
    const float ySpacing = gridHeight / (gridSize - 1);
    coarseStepX = calibrationBootstrap17 ? std::min(150.0f, gridWidth * 0.5f) : 150.0f;
    coarseStepY = calibrationBootstrap17 ? std::min(150.0f, gridHeight * 0.5f) : 150.0f;

    int numberOfCycles = 0;

    switch (gridSize) {
        case 3:
            numberOfCycles = 1;  // 3x3 grid
            break;
        case 5:
            numberOfCycles = 2;  // 5x5 grid
            break;
        case 7:
            numberOfCycles = 3;  // 7x7 grid
            break;
        case 9:
            numberOfCycles = 4;  // 9x9 grid
            break;
        default:
            log_error("Invalid " + M + "_calibration_grid_size: " << gridSize);
            return false;  // return false or handle error appropriately
    }

    pointCount         = 6;  //The first four points are computed dynamically
    recomputePoints[0] = 5;

    //The point in the center
    calibrationGrid[pointCount][0] = 0;
    calibrationGrid[pointCount][1] = 0;

    pointCount++;

    int maxX = 1;
    int maxY = 1;

    int currentX = 0;
    int currentY = -1;

    recomputeCount = 1;

    while (maxX <= numberOfCycles) {  //4 produces a 9x9 grid
        while (currentX > -1 * maxX) {
            calibrationGrid[pointCount][0] = currentX * xSpacing;
            calibrationGrid[pointCount][1] = currentY * ySpacing;
            pointCount++;
            currentX--;
        }
        while (currentY < maxY) {
            calibrationGrid[pointCount][0] = currentX * xSpacing;
            calibrationGrid[pointCount][1] = currentY * ySpacing;
            pointCount++;
            currentY++;
        }
        while (currentX < maxX) {
            calibrationGrid[pointCount][0] = currentX * xSpacing;
            calibrationGrid[pointCount][1] = currentY * ySpacing;
            pointCount++;
            currentX++;
        }
        while (currentY > -1 * maxY) {
            calibrationGrid[pointCount][0] = currentX * xSpacing;
            calibrationGrid[pointCount][1] = currentY * ySpacing;
            pointCount++;
            currentY--;
        }

        //Add the last point to the recompute list
        calibrationGrid[pointCount][0] = currentX * xSpacing;
        calibrationGrid[pointCount][1] = currentY * ySpacing;
        pointCount++;

        recomputePoints[recomputeCount] = pointCount - 1;  //Minus one because we increment after each point is generated
        recomputeCount++;

        maxX = maxX + 1;
        maxY = maxY + 1;

        currentY = currentY + -1;
    }

    //Move back to the center
    calibrationGrid[pointCount][0] = 0;
    calibrationGrid[pointCount][1] = (currentY + 1) * ySpacing;  //The last loop added an nunecessary -1 to the y position
    pointCount++;

    calibrationGrid[pointCount][0] = 0;
    calibrationGrid[pointCount][1] = 0;

    recomputePoints[recomputeCount] = pointCount;
    log_info("Find Anchors profile=" << (calibrationBootstrap17 ? "bootstrap17" : "precision")
             << " points=" << (pointCount + 1) << " grid=" << gridSize
             << " range=" << gridWidth << "x" << gridHeight << "mm"
             << " coarse_step=" << coarseStepX << "," << coarseStepY << "mm"
             << " repeatability=" << (calibrationBootstrap17 ? BOOTSTRAP17_REPEATABILITY_MM : 2.5)
             << "mm rms_limit=" << (calibrationBootstrap17 ? BOOTSTRAP17_RMS_LIMIT_MM : FITNESS_RMS_FAIL_MM)
             << "mm max_limit=" << (calibrationBootstrap17 ? BOOTSTRAP17_MAX_RESIDUAL_MM : FITNESS_MAX_RES_FAIL_MM) << "mm");
    return true;
}

/*
* This function takes a single measurement and adjusts the frame dimensions to find a valid frame size that matches the measurement
* Uses the algorithm described in https://math.stackexchange.com/questions/5013127/find-square-size-from-inscribed-triangles?noredirect=1#comment10752043_5013127 for calculating square size from distances to vertices
*/
bool Calibration::adjustFrameSizeToMatchFirstMeasurement() {
    const auto& measurements = measurementState.samples;
    //Get the last measurements
    double tlLen = measurements[0][0];  // distance to A (top-left)
    double trLen = measurements[0][1];  // distance to B (top-right)
    double blLen = measurements[0][2];  // distance to C (bottom-left)
    double brLen = measurements[0][3];  // distance to D (bottom-right)

    // Use the Stack Exchange algorithm to compute the square side length
    // The algorithm works for any interior point E inside the square
    double L = SquareCalculation::calculateSquareSideLength(tlLen, trLen, blLen, brLen);

    if (!std::isfinite(L) || L < 500.0 || L > 5500.0) {  // Sanity check on square size
        log_error("Unable to adjust frame size. Calculated size " << L << "mm is outside reasonable range");
        return false;
    }

    //Adjust the frame size to match the computed size
    auto kinematics = getKinematics();
    if (!kinematics) {
        log_error("adjustFrameSizeToMatchFirstMeasurement: MaslowKinematics not available");
        return false;
    }

    // Update the frame size in the MaslowKinematics system
    kinematics->setFrameSize(L);

    log_info("Frame size successfully adjusted to: " << L << " x " << L);
    return true;
}

//non-blocking delay, just pauses everything for specified time
void Calibration::hold(unsigned long time) {
    holdTime  = time;
    holding   = true;
    holdTimer = millis();
}

//Print calibration grid
// void Calibration::printCalibrationGrid() {
//     for (int i = 0; i <= pointCount; i++) {
//         log_info("Point " << i << ": " << calibrationGrid[i][0] << ", " << calibrationGrid[i][1]);
//     }
//     log_info("Max value for pointCount: " << pointCount);

//     for(int i = 0; i < recomputeCount; i++){
//         log_info("Recompute point: " << recomputePoints[i]);
//     }

//     log_info("Times to recompute: " << recomputeCount);

// }

//------------------------------------------------------
//------------------------------------------------------ Utility Functions
//------------------------------------------------------

//This function is used for release tension...is this function still needed?
void Calibration::comply() {
    complyCallTimer = millis();
    retracting[_TL] = false;
    retracting[_TR] = false;
    retracting[_BL] = false;
    retracting[_BR] = false;
    for (int arm = _TL; arm < ARM_COUNT; arm++) {
        Maslow.axis[arm].reset();  //This just resets the thresholds for pull tight
    }
}

// Direction from maslow current coordinates to the target coordinates
int Calibration::get_direction(double x, double y, double targetX, double targetY) {
    int direction = UP;

    if (targetX - x > 1) {
        direction = RIGHT;
    } else if (targetX - x < -1) {
        direction = LEFT;
    } else if (targetY - y > 1) {
        direction = UP;
    } else if (targetY - y < -1) {
        direction = DOWN;
    }

    return direction;
}

// Function to allocate memory for calibration arrays
void Calibration::allocateCalibrationMemory() {
    if (calibrationGrid == nullptr) {  //Check to prevent realocating
        calibrationGrid = new float[CALIBRATION_GRID_SIZE_MAX][2]{};
    }
    if (calibration_data == nullptr) {
        calibration_data = new float*[CALIBRATION_GRID_SIZE_MAX]{};
        for (int i = 0; i < CALIBRATION_GRID_SIZE_MAX; ++i) {
            calibration_data[i] = new float[4]{};
        }
    }
}

// Function to deallocate memory for calibration arrays
void Calibration::deallocateCalibrationMemory() {
    delete[] calibrationGrid;
    calibrationGrid = nullptr;
    if (calibration_data != nullptr) {
        for (int i = 0; i < CALIBRATION_GRID_SIZE_MAX; ++i) {
            delete[] calibration_data[i];
        }
        delete[] calibration_data;
        calibration_data = nullptr;
    }
}

void Calibration::resetMeasurementState() {
    measurementState = MeasurementState{};
    resetHorizontalSampling();
    horizontalSamplingReferenceInvalid = false;
    for (int arm = 0; arm < ARM_COUNT; ++arm) horizontalTight[arm] = false;
    horizontalInitialTensionComplete = false;
    horizontalMeasurementFault = false;
    horizontalHoldConflictStart = 0;
    orientationDetectionDone = false;
    orientationDetectTimer = 0;
    holding = false;
}

// Function to reset all calibration state variables to initial values
void Calibration::resetCalibrationState() {
    Maslow.stopMotors();
    savedAnchorHoldActive = false;
    applyTensionRecovery = ApplyTensionRecoveryState{};
    resetMeasurementState();
    diagnosticPhase = nullptr;
    diagnosticWaypoint = -1;
    for (int arm = _TL; arm < ARM_COUNT; ++arm) horizontalTight[arm] = false;
    horizontalInitialTensionComplete = false;
    horizontalMeasurementFault = false;
    horizontalHoldConflictStart = 0;
    // Reset calibration progress variables
    waypoint               = 0;
    pointCount             = 0;
    recomputeCountIndex    = 0;
    calibrationInProgress  = false;

    // Reset calibration loop state variables
    calibrationDirection  = UP;    // Default direction
    measurementInProgress = true;  // Start by taking a measurement

    // Reset orientation detection variables so it runs on next calibration
    orientationDetectionDone = false;
    orientationDetectTimer   = 0;

    // Reset fitness tracking so a fresh calibration run starts clean
    previousFitnessRms  = -1.0;
    lastRecomputePassed = false;

    // Deallocate memory if allocated
    deallocateCalibrationMemory();

    log_info("Find Anchors state reset");
}

//Takes a raw measurement, projects it into the XY plane, then adds the belt end extension and arm length to get the actual distance.
float Calibration::measurementToXYPlane(float measurement, float zHeight) {
    auto kinematics = getKinematics();
    if (!kinematics)
        return 0.0f;

    // Include spoilboard and work thickness in the total Z height to match normal operation
    // This ensures calibration measurements are consistent with actual cutting operations
    float totalZHeight = zHeight + kinematics->getSpoilboardThickness() + kinematics->getWorkThickness();

    // Validate that measurement is sufficient to reach the XY plane at this Z height
    // If totalZHeight >= measurement, the belt would be too short to reach the anchor point
    if (totalZHeight * totalZHeight >= measurement * measurement) {
        log_error("Invalid geometry: belt length " << measurement << " too short for Z height " << totalZHeight);
        return 0.0f;
    }

    float lengthInXY = sqrt(measurement * measurement - totalZHeight * totalZHeight);
    return lengthInXY + kinematics->getBeltEndExtension() +
           kinematics->getArmLength();  //Add the belt end extension and arm length to get the actual distance
}

//Takes an XY plane distance, subtracts the belt end extension and arm length, then calculates the angled belt measurement.
float Calibration::measurementFromXYPlane(float xyPlaneDistance, float zHeight) {
    auto kinematics = getKinematics();
    if (!kinematics)
        return 0.0f;

    // Include spoilboard and work thickness in the total Z height to match normal operation
    // This ensures calibration measurements are consistent with actual cutting operations
    float totalZHeight = zHeight + kinematics->getSpoilboardThickness() + kinematics->getWorkThickness();
    float lengthInXY =
        xyPlaneDistance - kinematics->getBeltEndExtension() - kinematics->getArmLength();  //Subtract the belt end extension and arm length

    // Validate that the XY plane distance is sufficient after subtracting mechanical offsets
    // Negative lengthInXY indicates an invalid configuration (router position too close to anchor)
    if (lengthInXY < 0.0f) {
        log_error("Invalid geometry: XY distance " << xyPlaneDistance << " too short for belt end extension + arm length");
        return 0.0f;
    }

    return sqrt(lengthInXY * lengthInXY + totalZHeight * totalZHeight);  //Calculate the angled belt length
}

/* Calculates and updates the center (X, Y) position based on the coordinates of the four corners
* (top-left, top-right, bottom-left, bottom-right) of a rectangular area. The center is determined
* by finding the intersection of the diagonals of the rectangle.
*/
void Calibration::updateCenterXY() {
    // The MaslowKinematics system handles center calculation automatically
    // We no longer maintain separate center coordinates in the Maslow class
    auto kinematics = getKinematics();
    if (kinematics) {
        // The center is already calculated in MaslowKinematics and accessible via getters
    }
}

// True if all axis were zeroed
bool Calibration::all_axis_homed() {
    return axis_homed[0] && axis_homed[1] && axis_homed[2] && axis_homed[3];
}

// True if all axis were extended
bool Calibration::allAxisExtended() {
    return extended[_TL] && extended[_TR] && extended[_BL] && extended[_BR];
}

bool Calibration::checkOverides() {
    if (TLIOveride || TRIOveride || BLIOveride || BRIOveride || TLOOveride || TROOveride || BLOOveride || BROOveride) {
        return true;
    }
    return false;
}

void Calibration::clearMotorOverrides() {
    TLIOveride = false;
    TRIOveride = false;
    BLIOveride = false;
    BRIOveride = false;
    TLOOveride = false;
    TROOveride = false;
    BLOOveride = false;
    BROOveride = false;
}

void Calibration::setSafety(bool state) {
    safetyOn = state;
}

//------------------------------------------------------
//------------------------------------------------------ Motor Overides
//------------------------------------------------------

//These are used to force one motor to rotate
void Calibration::TLI() {
    TLIOveride   = true;
    overideTimer = millis();
}
void Calibration::TRI() {
    TRIOveride   = true;
    overideTimer = millis();
}
void Calibration::BLI() {
    BLIOveride   = true;
    overideTimer = millis();
}
void Calibration::BRI() {
    BRIOveride   = true;
    overideTimer = millis();
}
void Calibration::TLO() {
    TLOOveride   = true;
    overideTimer = millis();
}
void Calibration::TRO() {
    TROOveride   = true;
    overideTimer = millis();
}
void Calibration::BLO() {
    BLOOveride   = true;
    overideTimer = millis();
}
void Calibration::BRO() {
    BROOveride   = true;
    overideTimer = millis();
}

/*
* This function is used to manuall force the motors to move for a fraction of a second to clear jams
*/
void Calibration::handleMotorOverides() {
    if (TLIOveride) {
        log_info(int(millis() - overideTimer));
        if (millis() - overideTimer < 200) {
            Maslow.axis[_TL].fullIn();
        } else {
            TLIOveride = false;
            Maslow.axis[_TL].stop();
        }
    }
    if (BRIOveride) {
        log_info(int(millis() - overideTimer));
        if (millis() - overideTimer < 200) {
            Maslow.axis[_BR].fullIn();
        } else {
            BRIOveride = false;
            Maslow.axis[_BR].stop();
        }
    }
    if (TRIOveride) {
        log_info(int(millis() - overideTimer));
        if (millis() - overideTimer < 200) {
            Maslow.axis[_TR].fullIn();
        } else {
            TRIOveride = false;
            Maslow.axis[_TR].stop();
        }
    }
    if (BLIOveride) {
        log_info(int(millis() - overideTimer));
        if (millis() - overideTimer < 200) {
            Maslow.axis[_BL].fullIn();
        } else {
            BLIOveride = false;
            Maslow.axis[_BL].stop();
        }
    }
    if (TLOOveride) {
        log_info(int(millis() - overideTimer));
        if (millis() - overideTimer < 200) {
            Maslow.axis[_TL].fullOut();
        } else {
            TLOOveride = false;
            Maslow.axis[_TL].stop();
        }
    }
    if (BROOveride) {
        log_info(int(millis() - overideTimer));
        if (millis() - overideTimer < 200) {
            Maslow.axis[_BR].fullOut();
        } else {
            BROOveride = false;
            Maslow.axis[_BR].stop();
        }
    }
    if (TROOveride) {
        log_info(int(millis() - overideTimer));
        if (millis() - overideTimer < 200) {
            Maslow.axis[_TR].fullOut();
        } else {
            TROOveride = false;
            Maslow.axis[_TR].stop();
        }
    }
    if (BLOOveride) {
        log_info(int(millis() - overideTimer));
        if (millis() - overideTimer < 200) {
            Maslow.axis[_BL].fullOut();
        } else {
            BLOOveride = false;
            Maslow.axis[_BL].stop();
        }
    }
}

/*
 * Detects the orientation (horizontal vs vertical) of the machine
 * by measuring TL and TR belt extension under gravity.
 * Returns true when detection is complete.
 */
bool Calibration::detectOrientation() {
    if (calibrationBootstrap17) {
        if (!orientationDetectionDone) {
            log_info("Find Anchors bootstrap17: using configured " << (orientation == VERTICAL ? "VERTICAL" : "HORIZONTAL") << " orientation");
            orientationDetectionDone = true;
        }
        return true;
    }
    const unsigned long STARTUP_DELAY_MS                = 50;    // Delay before starting test to ensure stable starting position
    const unsigned long ORIENTATION_DETECT_DURATION_MS  = 1500;  // Duration in ms to run orientation detection test (1.5 seconds)
    const float         ORIENTATION_DETECT_THRESHOLD_MM = 35.0;  // Minimum extension in mm to detect vertical orientation
    const int           ORIENTATION_DETECT_SPEED        = 716;   // PWM speed for motors (70% of max 1023)
    const unsigned long MOTOR_SETTLING_PAUSE_MS         = 500;   // Pause duration after retraction to allow motors to settle

    // Track whether the settling pause has completed
    bool& settlingCompleted = measurementState.settlingCompleted;

    // If settling has already completed, return true immediately on subsequent calls
    // This prevents re-running detection on subsequent calibration_loop iterations
    if (settlingCompleted) {
        return true;
    }

    // Initialize timer on first call
    if (orientationDetectTimer == 0) {
        orientationDetectTimer = millis();
        settlingCompleted      = false;  // Reset the flag when starting new detection
    }

    unsigned long elapsedTime = millis() - orientationDetectTimer;

    // Phase 1: Record starting positions (once at beginning)
    if (!orientationDetectionDone && elapsedTime < STARTUP_DELAY_MS) {
        diagnosticStage("orientation_start");
        // Only record and log on first cycle (elapsedTime will be very small)
        if (elapsedTime < 10) {  // First few milliseconds only
            tlStartPosition = Maslow.axis[_TL].getPosition();
            trStartPosition = Maslow.axis[_TR].getPosition();
        }
        return false;
    }

    // Phase 2: Actively drive TL and TR motors outward at 70% speed for 1.5 seconds
    // BL and BR motors are kept stopped (not powered)
    // In vertical orientation, gravity assists and belts extend significantly
    // In horizontal orientation, belts extend minimally despite motor drive
    if (!orientationDetectionDone && elapsedTime >= STARTUP_DELAY_MS && elapsedTime < (STARTUP_DELAY_MS + ORIENTATION_DETECT_DURATION_MS)) {
        diagnosticStage("orientation_extend");
        // Drive TL and TR motors at 70% speed in extend direction
        Maslow.axis[_TL].driveOut(ORIENTATION_DETECT_SPEED);
        Maslow.axis[_TR].driveOut(ORIENTATION_DETECT_SPEED);
        // Ensure BL and BR are stopped
        Maslow.axis[_BL].stop();
        Maslow.axis[_BR].stop();
        return false;
    }

    // Phase 3: Measure extension and return to starting positions
    if (!orientationDetectionDone && elapsedTime >= (STARTUP_DELAY_MS + ORIENTATION_DETECT_DURATION_MS)) {
        double tlCurrentPosition = Maslow.axis[_TL].getPosition();
        double trCurrentPosition = Maslow.axis[_TR].getPosition();

        double tlExtension  = tlCurrentPosition - tlStartPosition;
        double trExtension  = trCurrentPosition - trStartPosition;
        double avgExtension = (tlExtension + trExtension) / 2.0;

        // Determine orientation based on extension amount
        bool detectedOrientation = HORIZONTAL;
        if (avgExtension > ORIENTATION_DETECT_THRESHOLD_MM) {
            detectedOrientation = VERTICAL;
            log_info("Detected VERTICAL orientation (extension > " << ORIENTATION_DETECT_THRESHOLD_MM << " mm)");
        } else {
            detectedOrientation = HORIZONTAL;
            log_info("Detected HORIZONTAL orientation (extension <= " << ORIENTATION_DETECT_THRESHOLD_MM << " mm)");
        }

        // Update the calibration object's orientation
        // This will be reflected in the Maslow_vertical configuration parameter
        orientation = detectedOrientation;
        log_info("Orientation set to: " << (orientation == VERTICAL ? "VERTICAL" : "HORIZONTAL")
                                        << " (Maslow_vertical=" << (orientation ? "true" : "false") << ")");

        // Set targets to return to starting positions
        Maslow.axis[_TL].setTarget(tlStartPosition);
        Maslow.axis[_TR].setTarget(trStartPosition);

        log_info("CALDIAG orientation_return_start TL=" << tlStartPosition << " TR=" << trStartPosition
                 << " TL_now=" << tlCurrentPosition << " TR_now=" << trCurrentPosition);
        diagnosticStage("orientation_return");
        orientationDetectionDone = true;
        return false;
    }

    // Phase 4: Return to starting positions
    if (orientationDetectionDone) {
        // Use PID to return to starting positions
        Maslow.axis[_TL].recomputePID();
        Maslow.axis[_TR].recomputePID();

        double tlCurrentPosition = Maslow.axis[_TL].getPosition();
        double trCurrentPosition = Maslow.axis[_TR].getPosition();

        // Check if we've returned to starting positions (within 5mm tolerance)
        if (fabs(tlCurrentPosition - tlStartPosition) < 5.0 && fabs(trCurrentPosition - trStartPosition) < 5.0) {
            // Phase 5: Power down all motors and pause to allow settling
            // This prevents current spikes when immediately trying to pull belts tight in horizontal orientation
            unsigned long& settlingStartTime = measurementState.settlingStart;

            // Initialize settling timer on first entry to this phase
            if (settlingStartTime == 0) {
                settlingStartTime = millis();
            }

            // Power down all four motors during the settling pause
            for (int arm = _TL; arm < ARM_COUNT; arm++) {
                Maslow.axis[arm].stop();
            }

            // Check if settling pause is complete
            if (millis() - settlingStartTime >= MOTOR_SETTLING_PAUSE_MS) {
                settlingStartTime = 0;     // Reset for next calibration run
                log_info("CALDIAG orientation_return_complete");
                diagnosticStage("measurement_start");
                settlingCompleted = true;  // Mark settling as complete to prevent re-running detection
                return true;
            }

            return false;  // Continue settling pause
        }
    }

    return false;
}
