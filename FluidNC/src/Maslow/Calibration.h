#pragma once
#include <Arduino.h>
#include "MaslowEnums.h"
#include "CalibrationStabilityWindow.h"
#include "SavedAnchorRecovery.h"
#include "CalibrationHoldController.h"

class Calibration {
public:
    // Constructor
    Calibration();

    // Public method
    void home();
    void updateCenterXY();

    bool all_axis_homed();
    bool allAxisExtended();
    void safety_control();
    void update_frame_xyz();

    //Used to override and drive the motors directly
    void TLI();
    void TRI();
    void BLI();
    void BRI();
    void TLO();
    void TRO();
    void BLO();
    void BRO();
    void handleMotorOverides();
    bool checkOverides();
    void clearMotorOverrides();

    bool generate_calibration_grid();
    //void   printCalibrationGrid();
    bool  move_with_slack(double fromX, double fromY, double toX, double toY);
    int   get_direction(double x, double y, double targetX, double targetY);
    bool  take_measurement_avg_with_check(int waypoint, int dir);
    bool  take_measurement(float result[4], int dir, int run, int current, int waypoint);
    float measurementToXYPlane(float measurement, float zHeight);
    float measurementFromXYPlane(float xyPlaneDistance, float zHeight);
    bool  takeSlackFunc();
    bool  maintainSavedAnchorHold();
    bool  beginSavedAnchorMotionSettling();
    bool  savedAnchorMotionSettling() const { return savedAnchorHoldActive && applyTensionRecovery.motionSettling; }
    void  cancelSavedAnchorHold() { savedAnchorHoldActive = false; }
    bool  adjustFrameSizeToMatchFirstMeasurement();
    bool  computeXYfromLengths(double TL, double TR, float& x, float& y);
    void  calibration_loop();
    void allocateCalibrationMemory();
    void deallocateCalibrationMemory();
    void resetCalibrationState();

    void comply();
    bool detectOrientation();

    void hold(unsigned long time);

    void setSafety(bool state);

    //State machine functions
    int  getCurrentState();
    void printCurrentState();
    bool requestStateChange(int newState);

    // Set extended state variables (used when restoring from NVS)
    void setExtendedState(bool tl, bool tr, bool bl, bool br);

    //Public Variables
    //hold
    unsigned long holdTimer = millis();
    bool          holding   = false;
    unsigned long holdTime  = 0;

    //Public calibration state variables. These need to be public since they are accessed externally.
    //They probably shouldn't be.

    //Variables used by retraction
    int  retractCurrentThreshold = 1300;
    bool axisHomed[ARM_COUNT]    = { false, false, false, false };

    //Variables used by extension
    float extendDist = 1700;

    //Variables used by calibration
    bool  orientation = HORIZONTAL;
    float acceptableCalibrationThreshold = 0.5;
    float applyTensionBeltRetractionLimitMm = 300.0;
    bool  applyTensionAllowLimiting         = true;
    float applyTensionMaxResidualMm          = 2.5f;
    // First establish anchors with 17 samples; disable for the original precision/grid policy.
    bool  calibrationBootstrap17         = true;
    int   calibrationGridSize            = 9;
    float calibration_grid_width_mm_X    = 0;      // mm grid width (0 = auto-calculate as 50% of frame width)
    float calibration_grid_height_mm_Y   = 0;      // mm grid height (0 = auto-calculate as 20% of frame height)
    float calibrationMaxSpacingMm        = 260.0;  // Maximum allowed spacing between calibration points when auto-selecting grid size
    bool  calibrationInProgress = false;                   //Used to turn off regular movements during calibration

    //State machine variables
    int currentState = UNKNOWN;

private:
    // Diagnostic firmware: logging only; never changes motion or error state.
    void diagnosticStage(const char* stage);
    void diagnosticSnapshot();
    void diagnosticUpdate();
    const char* diagnosticPhase = nullptr;
    unsigned long diagnosticPhaseStart = 0;
    unsigned long diagnosticLastLog = 0;
    int diagnosticWaypoint = -1;
    int diagnosticRun = -1;
    int diagnosticCurrentThreshold = 0;
    unsigned diagnosticTightMask = 0;
    uint32_t diagnosticEncoderBaseline[ARM_COUNT] = {};
    bool horizontalTight[ARM_COUNT] = { false, false, false, false };
    bool horizontalInitialTensionComplete = false;
    bool horizontalMeasurementFault = false;
    unsigned long horizontalHoldConflictStart = 0;
    void logPendingBelts(bool extending);
    unsigned long beltOperationLogTime = 0;

    //Variables used for retracting state
    bool axis_homed[4]         = { false, false, false, false };
    bool retracting[ARM_COUNT] = { false, false, false, false };

    // An Apply Tension run owns its own references, timing and samples.
    // Find Anchors retains a separate acquisition path and fit policy.
    struct ApplyTensionRecoveryState {
        bool active = false;
        bool sampling = false;
        bool preloadResponding[ARM_COUNT] = {};
        MaslowSavedAnchorRecovery::PreloadDrive preload[ARM_COUNT];
        uint32_t started = 0, samplingStarted = 0, lastDiagnostic = 0, lastPreloadUpdate = 0;
        uint32_t holdConflictStarted = 0;
        bool holdConflict = false;
        bool motionSettling = false, motionWithinTolerance = false;
        uint32_t motionSettlingStarted = 0, motionWithinToleranceSince = 0;
        uint32_t longestMotionWithinToleranceMs = 0;
        double motionPeakError[ARM_COUNT] = {};
        unsigned motionOutsideToleranceMask = 0;
        unsigned samplePositionMask = 0, samplePwmMask = 0;
        MaslowCalibrationStability::Diagnostics sampleDiagnostics;
        float startLength[ARM_COUNT] = {};
        uint32_t readFailures[ARM_COUNT] = {};
        uint32_t largeSteps[ARM_COUNT] = {};
        int snapshots = 0, replacements = 0;
        float retained[4][ARM_COUNT] = {};
        MaslowCalibrationStability::Window stability;
    } applyTensionRecovery;
    bool savedAnchorHoldActive = false;
    bool beginApplyTension();
    bool failApplyTension(const char* reason, bool referenceInvalid);
    bool applyTensionFeedbackHealthy(uint32_t now);
    bool savedAnchorHoldSettled();
    bool keepSettledSavedAnchorHold();
    bool updateApplyTensionHold(uint32_t now);
    void logSavedAnchorSettling(const char* result, uint32_t now) const;
    void logSavedAnchorSampling(const char* result) const;

    // Store the previous state before entering RELEASE_TENSION
    int previousState = UNKNOWN;

    //Variables used by extending
    bool extended[ARM_COUNT] = { false, false, false, false };

    //Variables used by take slack
    bool takeSlack = false;

    //Variables used by calibration
    float** calibration_data      = nullptr;
    int     pointCount            = 0;     //number of actual points in the grid,  < GRID_SIZE_MAX
    int     waypoint              = 0;     //The current waypoint in the calibration process
    int     calibrationDirection  = 0;     //Direction for calibration measurements (replaces static variable)
    bool    measurementInProgress = true;  //Whether currently taking measurement or moving (replaces static variable)
    int     frame_dimention_MIN   = 400;   //Is this used? This should be enforced by the user settings. TODO.
    int     frame_dimention_MAX   = 15000;
    float (*calibrationGrid)[2]   = nullptr;
    int    recomputePoints[10];          // Stores the index of the points where we want to trigger a recompute
    int    recomputeCountIndex    = 0;   // Stores the index of the recompute point we are currently on
    int    recomputeCount         = 0;   // Stores the number of recompute points

    // Fitness tracking across recomputes
    double previousFitnessRms  = -1.0;  // RMS from prior recompute; -1.0 = no prior recompute
    bool   lastRecomputePassed = false;  // Set true only when all fitness gates pass

    // State belongs to a run, not to function statics: abort/retry must start clean.
    struct MeasurementState {
        int run = 0;
        int retries = 0;
        float samples[4][4] = {};
        bool bottomLeftTight = false;
        bool bottomRightTight = false;
        bool pull1Tight = false;
        bool pull2Tight = false;
        unsigned long moveBegin = 0;
        unsigned long moveLastUpdate = 0;
        double moveElapsedMs = 0;
        double moveBeltOffset[ARM_COUNT] = {};
        uint32_t moveDiagnosticLastSample = 0;
        uint32_t moveReadFailures[ARM_COUNT] = {};
        uint32_t moveLargeSteps[ARM_COUNT] = {};
        bool moveNeedsInit = true;
        int moveDirection = UP;
        float xStep = 0, yStep = 0;
        bool tlExtending = false, trExtending = false;
        bool blExtending = false, brExtending = false;
        unsigned long stabilizeTimer = 0;
        bool settlingCompleted = false;
        unsigned long settlingStart = 0;
    } measurementState;
    void diagnosticMoveTracking(bool withSlack);
    bool moveCalibratedHorizontal(double fromX, double fromY, double toX, double toY);
    void finishCalibrationMove(int direction);
    void resetMeasurementState();
    void resetHorizontalSampling();
    bool failHorizontalSampling(const char* reason, unsigned affectedBelts = 0, bool holding = false);
    bool takeStableHorizontalMeasurement(float result[4], int current, int point);
    struct HorizontalSamplingState {
        int point = -1;
        int firstBottom = _BR;
        bool holdingSamples = false;
        unsigned long phaseStarted = 0;
        uint32_t readFailures[ARM_COUNT] = {};
        uint32_t largeSteps[ARM_COUNT] = {};
        MaslowCalibrationStability::Window stability;
    } horizontalSampling;
    bool horizontalSamplingReferenceInvalid = false;
    float coarseStepX = 150.0f, coarseStepY = 150.0f;

    //Used to keep track of how often the PID controller is updated
    unsigned long lastCallToPID    = millis();
    unsigned long lastMiss         = millis();
    unsigned long lastCallToUpdate = millis();
    unsigned long extendCallTimer  = millis();
    unsigned long complyCallTimer  = millis();

    //Variables used for orientation detection
    unsigned long orientationDetectTimer   = 0;
    double        tlStartPosition          = 0;
    double        trStartPosition          = 0;
    bool          orientationDetectionDone = false;

    //Used to overide and drive motors directly...dangerous
    bool          TLIOveride   = false;
    bool          TRIOveride   = false;
    bool          BLIOveride   = false;
    bool          BRIOveride   = false;
    bool          TLOOveride   = false;
    bool          TROOveride   = false;
    bool          BLOOveride   = false;
    bool          BROOveride   = false;
    unsigned long overideTimer = millis();

    bool safetyOn         = true;

    bool recomputeAnchorsWithLevenbergMarquardt(int measurementCount);
    void logClbmMeasurements(int measurementCount) const;
    bool updateExtendDistanceFromAnchors();

    //A structure to hold the state names
    struct StateName {
        int         state;
        const char* name;
    };
    StateName stateNames[11] = { { UNKNOWN, "Unknown" },
                                 { RETRACTING, "Retracting Belts" },
                                 { RETRACTED, "Belts Retracted" },
                                 { EXTENDING, "Extending Belts" },
                                 { EXTENDEDOUT, "Belts Extended" },
                                 { TAKING_SLACK, "Taking Slack" },
                                 { CALIBRATION_IN_PROGRESS, "Finding Anchor" },
                                 { READY_TO_CUT, "Ready To Cut" },
                                 { RELEASE_TENSION, "Releasing Tension" },
                                 { CALIBRATION_COMPUTING, "Find Anchors Computing" } };
};
