// Copyright (c) 2024 - Maslow CNC. All rights reserved.
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "MaslowKinematics.h"

#include "../Machine/MachineConfig.h"
#include "../Limits.h"
#include "../Machine/Homing.h"
#include "../Protocol.h"
#include "../System.h"
#include <cstring>
#include "../NutsBolts.h"
#include "../MotionControl.h"
#include <cmath>
#include <algorithm>
#include "../Maslow/Maslow.h"
#include "../Maslow/SavedAnchorRecovery.h"

/*
Default configuration for Maslow CNC:

kinematics:
  MaslowKinematics:
    tlX: -27.6
    tlY: 2064.9
    tlZ: 100.0
    trX: 2924.3
    trY: 2066.5
    trZ: 56.0
    blX: 0.0
    blY: 0.0
    blZ: 34.0
    brX: 2953.2
    brY: 0.0
    brZ: 78.0
    beltEndExtension: 30.0
    armLength: 123.4
    beltToothSpacing: 1.9988  # mm between adjacent belt teeth
    encoderTeeth: 22.0        # tooth count on the encoder roller
    spoilboardThickness: 0.0
    workThickness: 0.0
    maxSegmentLength: 5.0

This implements the cable-driven kinematics for the Maslow CNC system.
The system has 5 axes:
- A, B, C, D (belt motors: TL, TR, BL, BR mapped to motors 0-3)
- Z (cartesian Z coordinate mapped to motor 4)

The spoilboardThickness and workThickness parameters account for material thickness
placed on the cutting surface. Both values are added to all anchor point Z coordinates
during belt length calculations, effectively raising the reference height for all belts.

The maxSegmentLength parameter controls belt length synchronization during long moves.
Moves longer than this distance (in mm) will be automatically segmented to ensure
correct belt lengths are computed at intermediate points, preventing belt slack.
*/

namespace Kinematics {

    // Global pointer to the current MaslowKinematics instance
    static MaslowKinematics* g_maslowKinematics = nullptr;

    void MaslowKinematics::init() {
        calculateCenter();
        g_maslowKinematics = this;  // Set global pointer for access
        init_position();
    }

    void MaslowKinematics::init_position() {
        auto n_axis = config->_axes->_numberAxis;
        for (size_t axis = 0; axis < n_axis; axis++) {
            set_motor_steps(axis, 0);  // Set to zeros
        }
    }

    void MaslowKinematics::calculateCenter() {
        // Calculate center point of the frame for coordinate system transformation
        // Find the intersection of the diagonals of the rectangle (proper geometric center)
        float A = (anchor_location[_TR][Coord_Y] - anchor_location[_BL][Coord_Y]) /
                  (anchor_location[_TR][Coord_X] - anchor_location[_BL][Coord_X]);
        float B = (anchor_location[_BR][Coord_Y] - anchor_location[_TL][Coord_Y]) /
                  (anchor_location[_BR][Coord_X] - anchor_location[_TL][Coord_X]);
        _centerX = (anchor_location[_BR][Coord_Y] - (B * anchor_location[_BR][Coord_X]) + (A * anchor_location[_TR][Coord_X]) -
                    anchor_location[_TR][Coord_Y]) /
                   (A - B);
        _centerY = A * (_centerX - anchor_location[_TR][Coord_X]) + anchor_location[_TR][Coord_Y];

        //log_info("Maslow center calculated: X=" << _centerX << " Y=" << _centerY);
    }

    bool MaslowKinematics::cartesian_to_motors(float* target, plan_line_data_t* pl_data, float* position) {
        auto n_axis = config->_axes->_numberAxis;

        // Ensure we have the expected number of axes (5: A, B, C, D, Z)
        if (n_axis < 5 || n_axis > MAX_N_AXIS) {
            log_error("MaslowKinematics requires between 5 and " << MAX_N_AXIS << " axes");
            return false;
        }

        // Apply work area constraints to the target position
        // Calculate work area bounds
        float halfWidth  = Maslow.workAreaX / 2.0f;
        float halfHeight = Maslow.workAreaY / 2.0f;
        float minX       = -halfWidth + Maslow.workAreaCenterOffsetX;
        float maxX       = halfWidth + Maslow.workAreaCenterOffsetX;
        float minY       = -halfHeight + Maslow.workAreaCenterOffsetY;
        float maxY       = halfHeight + Maslow.workAreaCenterOffsetY;

        // Constrain X and Y to work area
        if (target[X_AXIS] < minX) {
            target[X_AXIS] = minX;
        } else if (target[X_AXIS] > maxX) {
            target[X_AXIS] = maxX;
        }

        if (target[Y_AXIS] < minY) {
            target[Y_AXIS] = minY;
        } else if (target[Y_AXIS] > maxY) {
            target[Y_AXIS] = maxY;
        }

        float segmentStart[MAX_N_AXIS] = {};
        std::copy_n(position, n_axis, segmentStart);
        const float cartesianDistance = vector_distance(target, position, 3);
        const bool hasXYMotion = std::hypot(target[X_AXIS] - position[X_AXIS], target[Y_AXIS] - position[Y_AXIS]) >= 0.001f;
        int segmentCount = 1;
        if (!_isSegmenting && hasXYMotion && cartesianDistance > _maxSegmentLength) {
            segmentCount = static_cast<int>(std::min(1000.0f, std::ceil(cartesianDistance / _maxSegmentLength)));
            if (cartesianDistance > 100.0f && segmentCount > 100) {
                segmentCount = std::max(segmentCount, static_cast<int>(std::min(1000.0f, cartesianDistance / 2.0f)));
            }
        }

        plan_line_data_t motorPlan = *pl_data;
        if (!_isSegmenting && hasXYMotion && motorPlan.is_jog && !motorPlan.motion.rapidMotion &&
            !motorPlan.motion.inverseTime && Maslow.calibration.currentState == READY_TO_CUT) {
            motorPlan.feed_rate = std::min(motorPlan.feed_rate, Maslow.jogMaxFeed);
            log_info("Maslow XY jog profile=jog-fit-r1 requested_mm_min=" << pl_data->feed_rate
                     << " effective_mm_min=" << motorPlan.feed_rate
                     << " start_xy=" << position[X_AXIS] << ',' << position[Y_AXIS]
                     << " target_xy=" << target[X_AXIS] << ',' << target[Y_AXIS]
                     << " distance_mm=" << cartesianDistance);
        }
        if (motorPlan.motion.inverseTime) {
            // The planner converts inverse time into a speed for each motor block.
            motorPlan.feed_rate *= segmentCount;
        }

        if (segmentCount > 1) {
            _isSegmenting = true;
            for (int segment = 1; segment < segmentCount; ++segment) {
                const float fraction = static_cast<float>(segment) / segmentCount;
                float intermediate[MAX_N_AXIS] = {};
                for (size_t axis = 0; axis < n_axis; ++axis) {
                    intermediate[axis] = position[axis] + (target[axis] - position[axis]) * fraction;
                }

                if (!mc_linear(intermediate, &motorPlan, segmentStart)) {
                    _isSegmenting = false;
                    return false;
                }
                std::copy_n(intermediate, n_axis, segmentStart);
            }
            _isSegmenting = false;
        }

        float motors[MAX_N_AXIS] = {};
        transform_cartesian_to_motors(motors, target);
        const float segmentDistance = vector_distance(target, segmentStart, 3);
        if (!motorPlan.motion.rapidMotion && !motorPlan.motion.inverseTime && segmentDistance > 0.0f) {
            float startMotors[MAX_N_AXIS] = {};
            transform_cartesian_to_motors(startMotors, segmentStart);
            const float motorDistance = vector_distance(motors, startMotors, n_axis);
            // Give the Cartesian segment and its motor block the same nominal duration.
            if (motorDistance > 0.0f) {
                motorPlan.feed_rate *= motorDistance / segmentDistance;
            }
        }
        return mc_move_motors(motors, &motorPlan);
    }

    void MaslowKinematics::motors_to_cartesian(float* cartesian, float* motors, int n_axis) {
        /* 
        Forward kinematics for Maslow CNC - convert belt lengths back to X,Y,Z coordinates.
        
        With ABCDZX axis mapping:
        motors[0] = A axis = Top Left belt length
        motors[1] = B axis = Top Right belt length  
        motors[2] = C axis = Bottom Left belt length
        motors[3] = D axis = Bottom Right belt length
        motors[4] = Z axis = Router position
        motors[5] = X axis = (not used)
        */

        const float z = motors[4];
        cartesian[Z_AXIS] = z;
        const float effectiveZ = _fixedZ ? 0.0f : z;
        const float tlZ = effectiveZ + anchor_location[_TL][Coord_Z] + _spoilboardThickness + _workThickness;
        const float trZ = effectiveZ + anchor_location[_TR][Coord_Z] + _spoilboardThickness + _workThickness;
        const float tlDistance = measurementToXYPlane(motors[_TL], std::fabs(tlZ));
        const float trDistance = measurementToXYPlane(motors[_TR], std::fabs(trZ));

        float x = 0, y = 0;
        const bool haveSeed = computeXYfromBeltLengths(tlDistance, trDistance, x, y)
            && std::isfinite(x) && std::isfinite(y);
        cartesian[X_AXIS] = haveSeed ? x / Maslow.scaleX : 0;
        cartesian[Y_AXIS] = haveSeed ? y / Maslow.scaleY : 0;
        if (Maslow.calibration.currentState == READY_TO_CUT) {
            const double lengths[ARM_COUNT] = { motors[0], motors[1], motors[2], motors[3] };
            const auto predict = [this, z](int arm, double px, double py) {
                return compute(arm, px * Maslow.scaleX, py * Maslow.scaleY, z);
            };
            const auto fit = MaslowSavedAnchorRecovery::fitPosition(lengths, predict, []() {},
                cartesian[X_AXIS], cartesian[Y_AXIS]);
            if (fit.converged) {
                cartesian[X_AXIS] = fit.x;
                cartesian[Y_AXIS] = fit.y;
            }
        }

        // Only virtual XYZ are used. Initialize the remaining configured
        // Cartesian entries so callers cannot carry uninitialized A/B values.
        for (int axis = 3; axis < n_axis && axis < MAX_N_AXIS; axis++) {
            cartesian[axis] = 0.0f;
        }
    }

    void MaslowKinematics::transform_cartesian_to_motors(float* motors, float* cartesian) {
        // In this implementation, FluidNC axis order is ABCDZX:
        // motors[0] = A axis = Top Left belt length
        // motors[1] = B axis = Top Right belt length
        // motors[2] = C axis = Bottom Left belt length
        // motors[3] = D axis = Bottom Right belt length
        // motors[4] = Z axis = Router position
        // motors[5] = X axis = (not used, keep as 0)

        // Extract X, Y, Z coordinates from cartesian space and apply scale factors
        float x = cartesian[X_AXIS] * Maslow.scaleX;  // Apply X scale factor
        float y = cartesian[Y_AXIS] * Maslow.scaleY;  // Apply Y scale factor
        float z = cartesian[Z_AXIS];                  // Z_AXIS = 2 (no scaling for Z)

        // Check if belts are ready to cut - if not, don't compute belt movements
        // This allows the Z-axis to move independently when belts are not calibrated
        if (Maslow.calibration.currentState == READY_TO_CUT) {
            // Compute belt lengths for each corner and assign to correct axis
            for (int arm = _TL; arm < ARM_COUNT; ++arm) {
                motors[arm] = compute(arm, x, y, z);
            }
        } else {
            // When belts are not ready, keep them at their current positions
            // This prevents the motion planner from synchronizing Z-axis with large belt movements
            motors[0] = steps_to_mpos(get_axis_motor_steps(0), 0);  // Keep TL at current position
            motors[1] = steps_to_mpos(get_axis_motor_steps(1), 1);  // Keep TR at current position
            motors[2] = steps_to_mpos(get_axis_motor_steps(2), 2);  // Keep BL at current position
            motors[3] = steps_to_mpos(get_axis_motor_steps(3), 3);  // Keep BR at current position
        }

        motors[4] = z;  // Z position -> Z axis (pass through)

        auto n_axis = config->_axes->_numberAxis;
        // ABCDZ uses exactly five motor entries. Writing motors[5] on that
        // configuration overran the caller's five-element motion buffer.
        if (n_axis > 5) {
            motors[5] = 0.0f;  // Optional legacy sixth axis is unused.
        }

        // Handle any additional axes within the supported buffer capacity.
        for (size_t axis = 6; axis < n_axis && axis < MAX_N_AXIS; axis++) {
            motors[axis] = cartesian[axis];
        }
    }

    // Keep the pose fit's trial coordinates and objective in double precision.
    double MaslowKinematics::compute(int arm, double x, double y, double z) const {
        // Move from lower left corner coordinates to centered coordinates
        x       = x + _centerX;
        y       = y + _centerY;
        const double a = anchor_location[arm][Coord_X] - x;  // X dist from corner to router center
        const double b = anchor_location[arm][Coord_Y] - y;  // Y dist from corner to router center
        // When fixedZ is true, don't use current Z position - only use fixed anchor Z values
        const double effectiveZ = _fixedZ ? 0.0 : z;
        const double c          = 0.0f - (effectiveZ + anchor_location[arm][Coord_Z] + _spoilboardThickness +
                          _workThickness);  // Z dist from corner to router center (includes material thickness)

        const double xyBeltLength = std::sqrt(a * a + b * b) - (static_cast<double>(_beltEndExtension) + _armLength);
        return std::sqrt(xyBeltLength * xyBeltLength + c * c);
    }

    bool MaslowKinematics::canHome(AxisMask axisMask) {
        // For Maslow CNC, homing is typically done by retracting all belts
        // until they reach full retraction, then calibrating the system
        return true;
    }

    // Forward kinematics - compute X,Y from belt lengths
    bool MaslowKinematics::computeXYfromBeltLengths(float tlLength, float trLength, float& x, float& y) const {
        // Find the intersection of two circles centered at TL and TR anchor points
        // with radii equal to the belt lengths

        double d = sqrt((anchor_location[_TL][Coord_X] - anchor_location[_TR][Coord_X]) *
                            (anchor_location[_TL][Coord_X] - anchor_location[_TR][Coord_X]) +
                        (anchor_location[_TL][Coord_Y] - anchor_location[_TR][Coord_Y]) *
                            (anchor_location[_TL][Coord_Y] - anchor_location[_TR][Coord_Y]));
        if (d > tlLength + trLength || d < abs(tlLength - trLength)) {
            // Don't spam the console when belts are at zero length - this is expected behavior
            if (!(tlLength == 0.0f || trLength == 0.0f)) {
                log_info("Unable to determine machine position from belt lengths");
            }
            return false;
        }

        double a    = (tlLength * tlLength - trLength * trLength + d * d) / (2 * d);
        double h    = sqrt(tlLength * tlLength - a * a);
        double x0   = anchor_location[_TL][Coord_X] + a * (anchor_location[_TR][Coord_X] - anchor_location[_TL][Coord_X]) / d;
        double y0   = anchor_location[_TL][Coord_Y] + a * (anchor_location[_TR][Coord_Y] - anchor_location[_TL][Coord_Y]) / d;
        double rawX = x0 + h * (anchor_location[_TR][Coord_Y] - anchor_location[_TL][Coord_Y]) / d;
        double rawY = y0 - h * (anchor_location[_TR][Coord_X] - anchor_location[_TL][Coord_X]) / d;

        // Adjust to the centered coordinates (convert from frame coordinates to centered coordinates)
        x = rawX - _centerX;
        y = rawY - _centerY;

        return true;
    }

    // Convert angled belt measurement to XY plane distance
    float MaslowKinematics::measurementToXYPlane(float measurement, float zHeight) const {
        float squaredDiff = measurement * measurement - zHeight * zHeight;
        if (squaredDiff < 0.0f) {
            // Belt length is shorter than z-height component - geometrically invalid.
            // Return 0 so computeXYfromBeltLengths will detect an impossible configuration
            // and fall back to a safe default position instead of producing NaN.
            return 0.0f;
        }
        float lengthInXY = sqrtf(squaredDiff);
        return lengthInXY + _beltEndExtension + _armLength;  // Add belt end extension and arm length
    }

    void MaslowKinematics::releaseMotors(AxisMask axisMask, MotorMask motors) {
        // Release the specified motors
        // This is handled by the base motor system
    }

    bool MaslowKinematics::limitReached(AxisMask& axisMask, MotorMask& motors, MotorMask limited) {
        // For Maslow CNC, limits are based on the frame boundaries and belt lengths
        // This is handled by the motor system and limit switches
        return false;
    }

    void MaslowKinematics::group(Configuration::HandlerBase& handler) {
        handler.item("tlX", anchor_location[_TL][Coord_X]);
        handler.item("tlY", anchor_location[_TL][Coord_Y]);
        handler.item("tlZ", anchor_location[_TL][Coord_Z]);
        handler.item("trX", anchor_location[_TR][Coord_X]);
        handler.item("trY", anchor_location[_TR][Coord_Y]);
        handler.item("trZ", anchor_location[_TR][Coord_Z]);
        handler.item("blX", anchor_location[_BL][Coord_X]);
        handler.item("blY", anchor_location[_BL][Coord_Y]);
        handler.item("blZ", anchor_location[_BL][Coord_Z]);
        handler.item("brX", anchor_location[_BR][Coord_X]);
        handler.item("brY", anchor_location[_BR][Coord_Y]);
        handler.item("brZ", anchor_location[_BR][Coord_Z]);
        handler.item("beltEndExtension", _beltEndExtension);
        handler.item("armLength", _armLength);
        handler.item("beltToothSpacing", _beltToothSpacing);
        handler.item("encoderTeeth", _encoderTeeth);
        handler.item("maxSegmentLength", _maxSegmentLength);
        handler.item("fixedZ", _fixedZ);
    }

    void MaslowKinematics::afterParse() {
        // Recalculate center coordinates after configuration has been loaded
        // This is important when calibration results are saved and reloaded
        calculateCenter();
        Maslow.setEncoderGeometry(_beltToothSpacing, _encoderTeeth);
        log_debug("Center coordinates recalculated after configuration load: X=" << _centerX << " Y=" << _centerY);
    }

    // Setter methods for calibration system to update frame parameters
    void MaslowKinematics::setFrameSize(float frameSize) {
        // Update anchor coordinates for a square frame of size frameSize x frameSize
        // Bottom anchors at Y=0, Top anchors at Y=frameSize
        // Left anchors at X=0, Right anchors at X=frameSize
        // Keep the same Z coordinates

        // Set X coordinates: left anchors at 0, right anchors at frameSize
        anchor_location[_BL][Coord_X] = 0.0f;
        anchor_location[_TL][Coord_X] = 0.0f;
        anchor_location[_BR][Coord_X] = frameSize;
        anchor_location[_TR][Coord_X] = frameSize;

        // Set Y coordinates: bottom anchors at 0, top anchors at frameSize
        anchor_location[_BL][Coord_Y] = 0.0f;
        anchor_location[_BR][Coord_Y] = 0.0f;
        anchor_location[_TL][Coord_Y] = frameSize;
        anchor_location[_TR][Coord_Y] = frameSize;

        // Recalculate center coordinates
        calculateCenter();
    }

    void MaslowKinematics::setCalibrationAnchors(float tlX, float tlY, float trX, float trY, float brX) {
        anchor_location[_TL][Coord_X] = tlX;
        anchor_location[_TL][Coord_Y] = tlY;
        anchor_location[_TR][Coord_X] = trX;
        anchor_location[_TR][Coord_Y] = trY;
        anchor_location[_BL][Coord_X] = 0.0f;
        anchor_location[_BL][Coord_Y] = 0.0f;
        anchor_location[_BR][Coord_X] = brX;
        anchor_location[_BR][Coord_Y] = 0.0f;
        calculateCenter();
    }

    void MaslowKinematics::setSpoilboardThickness(float thickness) {
        if (_spoilboardThickness != thickness) {
            _spoilboardThickness = thickness;
            log_info("Spoilboard thickness set to " << thickness << " mm");
        }
    }

    void MaslowKinematics::setWorkThickness(float thickness) {
        if (_workThickness != thickness) {
            _workThickness = thickness;
            log_info("Work thickness set to " << thickness << " mm");
        }
    }

    void MaslowKinematics::validate() {
        validateAndCorrectAnchorCoordinates();
    }

void MaslowKinematics::validateAndCorrectAnchorCoordinates() {
    const float TOLERANCE            = 0.1f;  // Allow small floating point differences
    bool        coordinatesCorrected = false;

    // Default reasonable values (rounded for clarity that these are placeholder values)
    const float DEFAULT_TLX = -30.0f;
    const float DEFAULT_TLY = 2100.0f;
    const float DEFAULT_TRX = 2950.0f;
    const float DEFAULT_TRY = 2100.0f;
    const float DEFAULT_BLX = 0.0f;
    const float DEFAULT_BLY = 0.0f;
    const float DEFAULT_BRX = 3000.0f;
    const float DEFAULT_BRY = 0.0f;

    // Check that blX, blY, and brY should be zero (or very close to zero)
    if (std::abs(anchor_location[_BL][Coord_X]) > TOLERANCE) {
        log_warn("Bottom left X coordinate (blX) should be 0.0, but is " << anchor_location[_BL][Coord_X] << ". Correcting to 0.0.");
        anchor_location[_BL][Coord_X] = 0.0f;
        coordinatesCorrected          = true;
    }

    if (std::abs(anchor_location[_BL][Coord_Y]) > TOLERANCE) {
        log_warn("Bottom left Y coordinate (blY) should be 0.0, but is " << anchor_location[_BL][Coord_Y] << ". Correcting to 0.0.");
        anchor_location[_BL][Coord_Y] = 0.0f;
        coordinatesCorrected          = true;
    }

    if (std::abs(anchor_location[_BR][Coord_Y]) > TOLERANCE) {
        log_warn("Bottom right Y coordinate (brY) should be 0.0, but is " << anchor_location[_BR][Coord_Y] << ". Correcting to 0.0.");
        anchor_location[_BR][Coord_Y] = 0.0f;
        coordinatesCorrected          = true;
    }

    // Check that tlX < trX (left should be to the left of right)
    if (anchor_location[_TL][Coord_X] >= anchor_location[_TR][Coord_X]) {
        log_warn("Top left X coordinate (tlX=" << anchor_location[_TL][Coord_X] << ") should be less than top right X coordinate (trX="
                                               << anchor_location[_TR][Coord_X] << "). Correcting to reasonable defaults.");
        anchor_location[_TL][Coord_X] = DEFAULT_TLX;
        anchor_location[_TR][Coord_X] = DEFAULT_TRX;
        coordinatesCorrected          = true;
    }

    // Check that top points are above bottom points
    if (anchor_location[_TL][Coord_Y] <= anchor_location[_BL][Coord_Y] || anchor_location[_TR][Coord_Y] <= anchor_location[_BR][Coord_Y]) {
        char* buffer = getLogBuffer();
        snprintf(buffer,
                 1400,
                 "Top anchor points should be above bottom anchor points. tlY=%g should be > blY=%g, trY=%g should be > brY=%g. "
                 "Correcting to reasonable defaults.",
                 anchor_location[_TL][Coord_Y],
                 anchor_location[_BL][Coord_Y],
                 anchor_location[_TR][Coord_Y],
                 anchor_location[_BR][Coord_Y]);
        log_warn(buffer);
        releaseLogBuffer();
        anchor_location[_TL][Coord_Y] = DEFAULT_TLY;
        anchor_location[_TR][Coord_Y] = DEFAULT_TRY;
        coordinatesCorrected          = true;
    }

    // Check side lengths - minimum 500mm, maximum 5500mm
    const float MIN_SIDE_LENGTH = 500.0f;
    const float MAX_SIDE_LENGTH = 5500.0f;

    // Calculate distances for each side of the frame
    float topSideLength = sqrt(
        (anchor_location[_TR][Coord_X] - anchor_location[_TL][Coord_X]) * (anchor_location[_TR][Coord_X] - anchor_location[_TL][Coord_X]) +
        (anchor_location[_TR][Coord_Y] - anchor_location[_TL][Coord_Y]) * (anchor_location[_TR][Coord_Y] - anchor_location[_TL][Coord_Y]));
    float rightSideLength = sqrt(
        (anchor_location[_BR][Coord_X] - anchor_location[_TR][Coord_X]) * (anchor_location[_BR][Coord_X] - anchor_location[_TR][Coord_X]) +
        (anchor_location[_BR][Coord_Y] - anchor_location[_TR][Coord_Y]) * (anchor_location[_BR][Coord_Y] - anchor_location[_TR][Coord_Y]));
    float bottomSideLength = sqrt(
        (anchor_location[_BR][Coord_X] - anchor_location[_BL][Coord_X]) * (anchor_location[_BR][Coord_X] - anchor_location[_BL][Coord_X]) +
        (anchor_location[_BR][Coord_Y] - anchor_location[_BL][Coord_Y]) * (anchor_location[_BR][Coord_Y] - anchor_location[_BL][Coord_Y]));
    float leftSideLength = sqrt(
        (anchor_location[_TL][Coord_X] - anchor_location[_BL][Coord_X]) * (anchor_location[_TL][Coord_X] - anchor_location[_BL][Coord_X]) +
        (anchor_location[_TL][Coord_Y] - anchor_location[_BL][Coord_Y]) * (anchor_location[_TL][Coord_Y] - anchor_location[_BL][Coord_Y]));

    // Check if any side length is outside the valid range
    if (topSideLength < MIN_SIDE_LENGTH || topSideLength > MAX_SIDE_LENGTH || rightSideLength < MIN_SIDE_LENGTH ||
        rightSideLength > MAX_SIDE_LENGTH || bottomSideLength < MIN_SIDE_LENGTH || bottomSideLength > MAX_SIDE_LENGTH ||
        leftSideLength < MIN_SIDE_LENGTH || leftSideLength > MAX_SIDE_LENGTH) {
        // Frame dimensions are out of bounds - this is a critical error that cannot be auto-corrected
        // Operating with incorrect anchor points could damage the machine
        char* buffer = getLogBuffer();
        snprintf(buffer,
                 1400,
                 "Frame side lengths are outside valid range (500-5500mm). "
                 "Top=%gmm, Right=%gmm, Bottom=%gmm, Left=%gmm. Calibration cannot proceed with these dimensions.",
                 topSideLength,
                 rightSideLength,
                 bottomSideLength,
                 leftSideLength);
        log_error(buffer);
        releaseLogBuffer();
        String errorMsg = "Frame dimensions out of bounds. Top=" + String(topSideLength, 1) + "mm, Right=" + String(rightSideLength, 1) +
                          "mm, Bottom=" + String(bottomSideLength, 1) + "mm, Left=" + String(leftSideLength, 1) + "mm";
        Maslow.eStop(errorMsg);
    }

    // Sanity check for reasonable coordinate values (not negative for most coordinates, not excessively large)
    const float MAX_REASONABLE_COORD = 10000.0f;  // 10 meters should be more than enough for any Maslow frame

    if (anchor_location[_TL][Coord_Y] < 0 || anchor_location[_TR][Coord_Y] < 0 || anchor_location[_TL][Coord_Y] > MAX_REASONABLE_COORD ||
        anchor_location[_TR][Coord_Y] > MAX_REASONABLE_COORD || anchor_location[_BL][Coord_X] < 0 || anchor_location[_BR][Coord_X] < 0 ||
        anchor_location[_BR][Coord_X] > MAX_REASONABLE_COORD) {
        log_warn("Anchor coordinates contain unrealistic values. Resetting to reasonable defaults.");
        anchor_location[_TL][Coord_X] = DEFAULT_TLX;
        anchor_location[_TL][Coord_Y] = DEFAULT_TLY;
        anchor_location[_TR][Coord_X] = DEFAULT_TRX;
        anchor_location[_TR][Coord_Y] = DEFAULT_TRY;
        anchor_location[_BL][Coord_X] = DEFAULT_BLX;
        anchor_location[_BL][Coord_Y] = DEFAULT_BLY;
        anchor_location[_BR][Coord_X] = DEFAULT_BRX;
        anchor_location[_BR][Coord_Y] = DEFAULT_BRY;
        coordinatesCorrected          = true;
    }

    if (coordinatesCorrected) {
        char* buffer = getLogBuffer();
        snprintf(buffer,
                 1400,
                 "Anchor coordinates corrected. New values: tlX=%g tlY=%g trX=%g trY=%g blX=%g blY=%g brX=%g brY=%g",
                 anchor_location[_TL][Coord_X],
                 anchor_location[_TL][Coord_Y],
                 anchor_location[_TR][Coord_X],
                 anchor_location[_TR][Coord_Y],
                 anchor_location[_BL][Coord_X],
                 anchor_location[_BL][Coord_Y],
                 anchor_location[_BR][Coord_X],
                 anchor_location[_BR][Coord_Y]);
        log_info(buffer);
        releaseLogBuffer();
        // Recalculate center coordinates after correction
        calculateCenter();
    }
}

// Destructor - clear global pointer
MaslowKinematics::~MaslowKinematics() {
    if (g_maslowKinematics == this) {
        g_maslowKinematics = nullptr;
    }
}

// Configuration registration
namespace {
    KinematicsFactory::InstanceBuilder<MaslowKinematics> registration("MaslowKinematics");
}

// Global accessor function to get the current MaslowKinematics instance
MaslowKinematics* getMaslowKinematics() {
    return g_maslowKinematics;
}
}
