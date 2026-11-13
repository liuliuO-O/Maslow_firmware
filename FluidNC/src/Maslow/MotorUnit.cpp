// Copyright (c) 2024 Maslow CNC. All rights reserved.
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file with
// following exception: it may not be used for any reason by MakerMade or anyone with a business or personal connection to MakerMade

#include "MotorUnit.h"
#include "../Report.h"
#include "Maslow.h"
#include <cmath>

// PID controller tuning
#define P 300
#define I 0
#define D 0

//------------------------------------------------------
//------------------------------------------------------ Core utility functions
//------------------------------------------------------

void MotorUnit::begin(int forwardPin, int backwardPin, int readbackPin, int encoderAddress, int channel1, int channel2,
                      bool reverseEncoder) {
    _encoderAddress = encoderAddress;

    String encAddrLabel = Maslow.axis_id_to_label(_encoderAddress);

    Maslow.I2CMux.setPort(_encoderAddress);
    if (!encoder.begin()) {
        log_error("Encoder not found on " << encAddrLabel.c_str());
        Maslow.error        = true;
        Maslow.errorMessage = "Encoder not found on " + encAddrLabel;
    } else {
        log_info("Encoder connected on " << encAddrLabel.c_str());
    }
    // Apply software direction before zeroing so cumulative counts use the same angle convention.
    encoder.setDirection(reverseEncoder ? AS5600_COUNTERCLOCK_WISE : AS5600_CLOCK_WISE);
    if (reverseEncoder) {
        log_info(encAddrLabel.c_str() << " encoder software reversal enabled");
    }
    zero();

    motor.begin(forwardPin, backwardPin, readbackPin, channel1, channel2);

    positionPID.setPID(P, I, D);
    positionPID.setOutputLimits(-1023, 1023);

    if (!motor_test()) {
        log_error("Motor not found on " << encAddrLabel.c_str());
        Maslow.error        = true;
        Maslow.errorMessage = "Motor not found on " + encAddrLabel;
    } else {
        log_info("Motor detected on " << encAddrLabel.c_str());
    }
}

//Test the motor unit by testing the motor and checking the encoder
bool MotorUnit::test() {
    bool allTestsPassed = true;

    String encAddrLabel = Maslow.axis_id_to_label(_encoderAddress);

    //Check if the motor / motor driver are connected
    if (!motor_test()) {
        log_warn("Motor not found on " << encAddrLabel.c_str());
        Maslow.error        = true;
        Maslow.errorMessage = "Motor not found on " + encAddrLabel;
        allTestsPassed      = false;
    }

    //Check if the encoder is connected
    if (!updateEncoderPosition()) {
        log_warn("Encoder not found on " << encAddrLabel.c_str());
        Maslow.error        = true;
        Maslow.errorMessage = "Encoder not found on " + encAddrLabel;
        allTestsPassed      = false;
    }

    //Check for the presence of the magnet
    if (!encoder.detectMagnet()) {
        log_warn("Magnet not detected on " << encAddrLabel.c_str());
        allTestsPassed = false;
    }

    if (allTestsPassed) {
        log_info("All tests passed on " << encAddrLabel.c_str());
    }

    return !Maslow.error;
}

bool MotorUnit::motor_test() {
    //run motor for 100ms and check if the current gets above zero
    unsigned long time        = millis();
    unsigned long elapsedTime = millis() - time;
    while (elapsedTime < 100) {
        elapsedTime = millis() - time;
        motor.forward(1023);
        if (motor.readCurrent() > 30) {
            motor.stop();
            return true;
        }
    }
    motor.stop();
    return false;
}
// update the motor current buffer and belts speed every >5 ms
void MotorUnit::update() {
    //updating belt speed and motor cutrrent
    //update belt speed every 50ms or so:
    if (millis() - beltSpeedTimer > 50) {
        beltSpeed             = (getPosition() - beltSpeedLastPosition) / ((millis() - beltSpeedTimer) / 1000.0);  // mm/s
        beltSpeedTimer        = millis();
        beltSpeedLastPosition = getPosition();
    }

    if (millis() - motorCurrentTimer > 5) {
        motorCurrentTimer = millis();
        for (int i = 0; i < 9; i++) {
            motorCurrentBuffer[i] = motorCurrentBuffer[i + 1];
        }
        motorCurrentBuffer[9] = motor.readCurrent();
    }

    // Drive-loss capture, checked every 100 ms.  Signature: |pwm|>=100 while
    // averaged current <=5 ADC and belt moves <0.5 mm per check.  Sustained for
    // 500 ms it latches and logs one snapshot; recovery logs once.  Thresholds
    // intentionally strict so free-spinning slack belts (moving, or low pwm)
    // do not trigger it.  Diagnostic only, motor outputs are untouched.
    const unsigned long nowMs = millis();
    if (nowMs - driveLossCheckMs >= 100) {
        const unsigned long dtMs = driveLossCheckMs ? (nowMs - driveLossCheckMs) : 100;
        const double        pos  = getPosition();
        if (!driveLossPosInit) {
            driveLossLastPos = pos;
            driveLossPosInit = true;
        }
        const double pwm = _commandPWM;
        const double cur = getMotorCurrent();
        // Rolling 1.2 s history so a latched event can show whether the current
        // collapsed abruptly (protection trip / supply open) or decayed
        // (connector heating, thermal fold-back).
        driveHist[driveHistPos] = { static_cast<int16_t>(constrain(pwm, -32768.0, 32767.0)),
                                    static_cast<uint16_t>(constrain(cur, 0.0, 65535.0)) };
        driveHistPos            = (driveHistPos + 1) % driveHistLen;
        if (driveHistFill < driveHistLen) ++driveHistFill;
        if (fabs(pwm) < 50.0) driveLossIdleCurrent = cur;
        const bool   suspect =
            fabs(pwm) >= 100.0 && cur <= 5.0 && fabs(pos - driveLossLastPos) < 0.5;
        if (suspect) {
            if (!driveLossActive) {
                driveLossRunMs += dtMs;
                if (driveLossRunMs >= 500) {
                    driveLossActive = true;
                    ++driveLossEvents;
                    log_warn("CALDIAG drive_loss axis=" << Maslow.axis_id_to_label(_encoderAddress).c_str()
                             << " event=" << driveLossEvents << " pwm=" << pwm << " current_avg=" << cur
                             << " current_raw=" << motor.readCurrent() << " pos=" << pos << " target=" << setpoint
                             << " belt_speed=" << beltSpeed << " enc_fail_total=" << encoderReadFailures
                             << " idle_cur=" << driveLossIdleCurrent);
                    String hist;
                    for (int i = 0; i < driveHistFill; ++i) {
                        const int idx = (driveHistPos + driveHistLen - driveHistFill + i) % driveHistLen;
                        hist += String(driveHist[idx].pwm);
                        hist += '/';
                        hist += String(driveHist[idx].cur);
                        if (i + 1 < driveHistFill) hist += ' ';
                    }
                    log_warn("CALDIAG drive_loss hist_100ms pwm/cur oldest_to_newest: " << hist.c_str());
                    log_warn("CALDIAG drive_loss hint: keep power connected; measure driver VM and motor "
                             "terminal voltage before any cable swap, swap clears the fault state");
                }
            }
        } else {
            if (driveLossActive) {
                log_warn("CALDIAG drive_loss_recovered axis=" << Maslow.axis_id_to_label(_encoderAddress).c_str()
                         << " event=" << driveLossEvents << " pwm=" << pwm << " current_avg=" << cur
                         << " pos=" << pos);
            }
            driveLossActive = false;
            driveLossRunMs  = 0;
        }
        driveLossLastPos = pos;
        driveLossCheckMs = nowMs;
    }
}

// Reads the encoder value and updates it's position
bool MotorUnit::updateEncoderPosition() {
    if (!Maslow.I2CMux.setPort(_encoderAddress)) {
        ++encoderReadFailures;
        return false;
    }

    String encAddrLabel = Maslow.axis_id_to_label(_encoderAddress);

    const int32_t cumulativeCounts = encoder.getCumulativePosition();

    // Check the angle transaction directly.  A separate isConnected() probe on
    // every sample doubled I2C traffic and reduced the effective sample rate.
    int error = encoder.lastError();
    if (error != 0) {  // AS5600_OK = 0
        ++encoderReadFailures;
        if (millis() - encoderReadFailurePrintTime > 5000) {
            encoderReadFailurePrintTime = millis();
            log_warn("Encoder I2C error " << error << " on " << encAddrLabel.c_str());
        }
        return false;
    }

    mostRecentCumulativeEncoderReading = cumulativeCounts;
    const unsigned long now = millis();
    encoderSampleMillis = static_cast<uint32_t>(now);
    ++encoderSampleSequence;
    if (encoderDiagnosticInitialized) {
        const int32_t countDelta = cumulativeCounts - encoderDiagnosticLastCounts;
        if (abs(countDelta) > 1536) {
            ++encoderLargeSteps;
            ++encoderDiagnosticSuppressed;
            if (now - encoderDiagnosticLastPrintTime >= 250) {
                // These extra I2C reads are diagnostic only.  Keep them out of
                // the normal sampling path so they cannot reduce encoder rate.
                const uint8_t magnetStatus = encoder.readStatus();
                int diagnosticI2CError     = encoder.lastError();
                int agc                    = -1;
                int magnitude              = -1;
                if (diagnosticI2CError == AS5600_OK) {
                    agc                = encoder.readAGC();
                    diagnosticI2CError = encoder.lastError();
                }
                if (diagnosticI2CError == AS5600_OK) {
                    magnitude          = encoder.readMagnitude();
                    diagnosticI2CError = encoder.lastError();
                }

                log_warn("CALDIAG encoder_step axis=" << encAddrLabel.c_str()
                         << " sensor_raw=" << encoder.getLastSensorAngle()
                         << " directed=" << encoder.getLastReadAngle()
                         << " direction=" << static_cast<unsigned int>(encoder.getDirection())
                         << " mux_expected=" << _encoderAddress
                         << " delta_counts=" << countDelta
                         << " cumulative_counts=" << cumulativeCounts
                         << " dt_ms=" << static_cast<unsigned int>(now - encoderDiagnosticLastTime)
                         << " pwm=" << _commandPWM
                         << " magnet_status=" << static_cast<unsigned int>(magnetStatus)
                         << " magnet_detected=" << static_cast<unsigned int>((magnetStatus & 0x20) != 0)
                         << " magnet_high=" << static_cast<unsigned int>((magnetStatus & 0x08) != 0)
                         << " magnet_low=" << static_cast<unsigned int>((magnetStatus & 0x10) != 0)
                         << " agc=" << agc
                         << " magnitude=" << magnitude
                         << " diag_i2c_error=" << diagnosticI2CError
                         << " suppressed=" << encoderDiagnosticSuppressed);
                encoderDiagnosticLastPrintTime = now;
                encoderDiagnosticSuppressed = 0;
            }
        }
    }
    encoderDiagnosticInitialized = true;
    encoderDiagnosticLastCounts  = cumulativeCounts;
    encoderDiagnosticLastTime    = now;
    return true;
}

/*!
 *  @brief  Gets the current error in the axis position
 */
double MotorUnit::getPositionError() {
    return getPosition() - setpoint;
}

// Recomputes the PID and drives the output
double MotorUnit::recomputePID() {
    calibrationHold.reset();
    _commandPWM = positionPID.getOutput(getPosition(), setpoint);

    motor.runAtPWM(_commandPWM);

    return _commandPWM;
}

// Use encoder-paced position feedback with the existing continuous friction compensation.
bool MotorUnit::recomputeMotionPID() {
    const double position = getPosition();
    if (!hasFreshEncoderFeedback(static_cast<uint32_t>(millis())) || !std::isfinite(position) || !std::isfinite(setpoint)) {
        stop();
        return false;
    }
    if (motionPIDSequence == encoderSampleSequence) return true;

    motionPIDSequence = encoderSampleSequence;
    calibrationHold.reset();
    const double effort = positionPID.getOutput(position, setpoint);
    _commandPWM = std::lround(MaslowCalibrationHold::Controller::driveForEffort(effort, 1023.0));
    motor.runAtPWMDirect(static_cast<long>(_commandPWM));
    return true;
}

//------------------------------------------------------
//------------------------------------------------------ Homing/calibration functions
//------------------------------------------------------

bool MotorUnit::recomputeCalibrationHold() {
    if (!hasFreshEncoderFeedback(static_cast<uint32_t>(millis()))) {
        stop();
        return false;
    }
    if (calibrationHold.active() && calibrationHoldSequence == encoderSampleSequence) return true;

    if (!calibrationHold.update(encoderSampleMillis, getPosition(), setpoint)) {
        stop();
        return false;
    }
    calibrationHoldSequence = encoderSampleSequence;
    _commandPWM = std::lround(calibrationHold.output());
    motor.runAtPWMDirect(static_cast<long>(_commandPWM));
    return true;
}

// Sets the motor to comply with how it is being pulled, non-blocking.
bool MotorUnit::comply() {
    //Call it every 25 ms
    if (millis() - lastCallToComply < 25) {
        return true;
    }

    //If we've moved any, then drive the motor outwards to extend the belt
    float positionNow = getPosition();
    float distMoved   = positionNow - lastPosition;

    //If the belt is moving out, let's keep it moving out
    if (distMoved > .1) {  //EXPERIMENTAL

        motor.forward(amtToMove);

        if (amtToMove < 100)
            amtToMove = 100;
        amtToMove = amtToMove * 1.4;

        amtToMove = min(amtToMove, 1023.0);
    }

    //Finally if the belt is not moving we want to spool things down
    else {
        amtToMove = amtToMove / 1.25;
        motor.forward(amtToMove);
    }

    _commandPWM  = amtToMove;  //update actual motor power, so the get motor power isn't lying to us
    lastPosition = positionNow;

    lastCallToComply = millis();
    return true;
}

// Pulls_tight and zeros axis; returns true when done
bool MotorUnit::retract() {
    if (!pull_tight(Maslow.calibration.retractCurrentThreshold))
        return false;

    String encAddrLabel = Maslow.axis_id_to_label(_encoderAddress);
    log_info("收带 | " << encAddrLabel.c_str() << " 已拉紧并归零");
    zero();

    return true;
}

// Pulls the belt until we hit a current treshold; returns true when done
bool MotorUnit::pull_tight(int currentThreshold) {
    const unsigned long now = millis();
    //call at most every 5ms
    if (now - lastCallToRetract < 4) {
        return false;
    }
    lastCallToRetract = now;

    // A latched fault means this belt cannot be tensioned any further.  Holding
    // the bridge at zero is what keeps a jam from becoming a sustained 100% duty
    // command, which is what trips the DRV8873 over-current / thermal limit and
    // then looks like "no current, no motion, full pwm" in the logs.
    if (tightFault()) {
        motor.stop();
        _commandPWM = 0;
        return false;
    }

    // Once tight, hold the duty that produced that decision instead of ramping up
    // from zero again.  Every re-ramp used to add another inward nudge every
    // ~130 ms for as long as a neighbouring belt was still tightening.
    if (tightOutcome_ == TIGHT_OK) {
        motor.backward(tightHoldPwm);
        _commandPWM = -tightHoldPwm;
        return false;
    }

    //Gradually increase the pulling speed
    if (random(0, 2) == 1) {
        retract_speed = min(retract_speed + 1, 1023);
    }

    motor.backward(retract_speed);
    _commandPWM = -retract_speed;  //This is only used for the getPWM function. There's got to be a tidier way to do this.

    //When taught
    int currentMeasurement = motor.readCurrent();

    retract_baseline = alpha * float(currentMeasurement) + (1 - alpha) * retract_baseline;

    if (currentMeasurement - retract_baseline > incrementalThreshold) {
        incrementalThresholdHits = incrementalThresholdHits + 1;
    } else {
        incrementalThresholdHits = 0;
    }

    // Stall guard.  A belt that is genuinely tight still drives the current up to
    // the configured threshold; a jam sits well below it and a driver that has
    // already shut down shows no current at all.  Sampled every 100 ms and judged
    // only after the window closes, so the ADC averaging buffer is settled.
    const double positionNow = getPosition();
    if (stallCheckMs == 0 || now - stallCheckMs >= 100) {
        const uint32_t windowMs = stallCheckMs ? static_cast<uint32_t>(now - stallCheckMs) : 0;
        const double   movedMm  = stallCheckMs ? std::fabs(positionNow - stallCheckPos) : 0.0;
        if (retract_speed >= stallMinPwm && windowMs > 0 && movedMm < stallMinMotionMm) {
            stallMs += windowMs;
        } else {
            stallMs = 0;
        }
        stallCheckPos = positionNow;
        stallCheckMs  = now;
        if (stallMs >= stallWindowMs) {
            // Near full duty with no travel.  The tension tests above have not
            // fired, so more duty cannot produce tension: only heat.  A current
            // already close to the threshold is left alone (it is about to pass),
            // which also keeps machines whose belts tension below the configured
            // threshold working exactly as before.
            const double meanCurrent = getMotorCurrent();
            if (meanCurrent < 0.8 * currentThreshold) {
                const bool noDrive = meanCurrent <= 5.0;
                const int  pwmAtFault = -_commandPWM;
                tightOutcome_        = noDrive ? TIGHT_NO_DRIVE : TIGHT_STALLED;
                motor.stop();
                _commandPWM      = 0;
                retract_speed    = 0;
                retract_baseline = 700;
                log_warn("CALDIAG tight_fault axis=" << Maslow.axis_id_to_label(_encoderAddress).c_str()
                         << " reason=" << (noDrive ? "no_drive" : "stall")
                         << " pwm=" << pwmAtFault << " current_avg=" << meanCurrent
                         << " current_raw=" << currentMeasurement << " threshold=" << currentThreshold
                         << " baseline=" << retract_baseline << " frozen_ms=" << static_cast<unsigned>(stallMs)
                         << " motion_mm=" << movedMm << " pos=" << positionNow
                         << " note=output_suppressed_until_reset");
                return false;
            }
        }
    }

    if (retract_speed > 15) {  //20 is not the actual speed, it is the amount of time so we don't trigger immediately
        // Decision unchanged: the incremental test still reports a taut belt on
        // this machine (observed ~0.7 x threshold), so raising a current floor
        // here would stop normal retraction from ever completing.
        const bool absolute = currentMeasurement > currentThreshold;
        const bool increment = incrementalThresholdHits > 2;
        if (absolute || increment) {
            // Capture the values used by the original decision before stop/reset.
            // Report only after stopping; do not add ADC reads or change the condition.
            const double diagnosticPWM = _commandPWM;
            const float diagnosticBaseline = retract_baseline;
            const int diagnosticHits = incrementalThresholdHits;
            const bool diagnosticAbsolute = absolute;
            const bool diagnosticIncrement = increment;
            const uint16_t holdPwm = retract_speed < tightHoldPwmCap ? retract_speed : tightHoldPwmCap;
            //stop motor, reset variables
            stop();
            retract_speed    = 0;
            retract_baseline = 700;
            tightOutcome_    = TIGHT_OK;
            tightHoldPwm     = holdPwm;
            // One INFO line per belt per tensioning attempt: the operator needs the
            // decision and the held duty without turning on Debug (Debug also emits
            // CALDIAG trace every 500 ms during calibration).
            log_info("拉紧成功 | " << Maslow.axis_id_to_label(_encoderAddress).c_str()
                     << " | 电流 " << currentMeasurement << " / 阈值 " << currentThreshold
                     << " | 判据 "
                     << (diagnosticAbsolute ? (diagnosticIncrement ? "绝对+增量" : "绝对电流")
                                            : "电流增量")
                     << " | 保持占空比 " << holdPwm << " (上限 " << tightHoldPwmCap << ")");
            log_debug("CALDIAG tight_success axis=" << Maslow.axis_id_to_label(_encoderAddress).c_str()
                     << " reason=" << (diagnosticAbsolute ? (diagnosticIncrement ? "both" : "absolute") : "increment")
                     << " adc=" << currentMeasurement << " threshold=" << currentThreshold
                     << " baseline=" << diagnosticBaseline
                     << " delta=" << (currentMeasurement - diagnosticBaseline)
                     << " delta_threshold=" << incrementalThreshold << " hits=" << diagnosticHits
                     << " pwm_before_stop=" << diagnosticPWM << " hold_pwm=" << holdPwm << " pos=" << getPosition());
            return true;
        } else {
            return false;
        }
    }
    return false;
}

// extends the belt to the target length until it hits the target length, returns true when target length is reached
bool MotorUnit::extend(double targetLength) {
    //unsigned long timeLastMoved = millis();

    if (getPosition() < targetLength) {
        comply();  //Comply does the actual moving
        return false;
    }
    //If reached target position, Stop and return
    setTarget(getPosition());
    stop();

    //log_info("Belt positon after extend: ");
    //log_info(getPosition());
    return true;
}

//------------------------------------------------------
//------------------------------------------------------ Utility functions
//------------------------------------------------------

// Sets the target location in mm
void MotorUnit::setTarget(double newTarget) {
    setpoint = newTarget;
}

// Gets the target location in mm
double MotorUnit::getTarget() {
    return setpoint;
}

// Returns the current position of the axis in mm
double MotorUnit::getPosition() {
    double positionNow = (mostRecentCumulativeEncoderReading / 4096.0) * _mmPerRevolution * -1;
    return positionNow;
}

// Returns the current motor power draw
double MotorUnit::getCurrent() {
    return motor.readCurrent();
}

// Stops the motor
void MotorUnit::stop() {
    calibrationHoldSequence = 0;
    motionPIDSequence = 0;
    calibrationHold.reset();
    motor.stop();
    _commandPWM = 0;
}

void MotorUnit::driveIn(int pwm) {
    _commandPWM = -pwm;
    motor.runAtPWMDirect(-pwm);
}

// Returns the PWM values set to the motor
double MotorUnit::getMotorPower() {
    return _commandPWM;
}

// Returns current belts speed, remove? (TODO)
double MotorUnit::getBeltSpeed() {
    return beltSpeed;
}

// Returns average motor current over last 10 reads
double MotorUnit::getMotorCurrent() {
    //return average motor current of the last 10 readings:
    double sum = 0;
    for (int i = 0; i < 10; i++) {
        sum += motorCurrentBuffer[i];
    }
    return sum / 10.0;
}

// Checking if we are at the target position within certain precision:
bool MotorUnit::onTarget(double precision) {
    if (abs(getTarget() - getPosition()) < precision)
        return true;
    else
        return false;
}

//Runs the motor to extend at full speed
void MotorUnit::decompressBelt() {
    int decompressSpeed = 800;
    motor.forward(decompressSpeed);
    _commandPWM = decompressSpeed;
}

//Drives the motor outward at a specified PWM speed
void MotorUnit::driveOut(int speed) {
    motor.forward(speed);
    _commandPWM = speed;
}

//Runs the motor at full speed out
void MotorUnit::fullOut() {
    motor.fullOut();
    _commandPWM = 1023;
}
//Runs the motor at full speed in
void MotorUnit::fullIn() {
    motor.fullIn();
    _commandPWM = -1023;  // inward: keep the reported duty consistent with driveIn()
}

// Reset all the axis variables
void MotorUnit::reset() {
    retract_speed            = 0;
    retract_baseline         = 700;
    incrementalThresholdHits = 0;
    amtToMove                = 0;
    lastPosition             = getPosition();
    beltSpeedTimer           = millis();
    // A new tensioning phase (retract, calibration waypoint, release) starts with
    // a clean latch; the fault only suppresses output until the caller resets.
    tightOutcome_  = TIGHT_PULLING;
    tightHoldPwm   = 0;
    stallCheckMs   = 0;
    stallCheckPos  = lastPosition;
    stallMs        = 0;
}

// Resets PID state to clear any accumulated integral term.
// Call this after a hard stop to prevent residual I-term motion.
void MotorUnit::resetPID() {
    calibrationHoldSequence = 0;
    motionPIDSequence = 0;
    calibrationHold.reset();
    positionPID.reset();
}

//sets the encoder position to 0
void MotorUnit::zero() {
    Maslow.I2CMux.setPort(_encoderAddress);
    encoder.resetCumulativePosition();
    // Update our cached value to match the reset position
    mostRecentCumulativeEncoderReading = 0;
    encoderDiagnosticInitialized       = false;
}

//sets the encoder position to a specific value (for restoring from NVS)
void MotorUnit::setPosition(double position) {
    // Convert position in mm to encoder counts
    // Formula: encoderCounts = -(position_mm * 4096.0) / _mmPerRevolution
    int32_t encoderCounts = (int32_t)(-(position * 4096.0) / _mmPerRevolution);

    Maslow.I2CMux.setPort(_encoderAddress);
    encoder.resetCumulativePosition(encoderCounts);

    // Update our cached value
    mostRecentCumulativeEncoderReading = encoderCounts;
    encoderDiagnosticInitialized = false;
}

//Gets the current raw encoder angle (0-4095)
uint16_t MotorUnit::getRawEncoderAngle() {
    Maslow.I2CMux.setPort(_encoderAddress);
    return encoder.readAngle();
}

void MotorUnit::setEncoderGeometry(double beltToothSpacing, double encoderTeeth) {
    if (!std::isfinite(beltToothSpacing) || !std::isfinite(encoderTeeth) || beltToothSpacing <= 0.0 || encoderTeeth <= 0.0) {
        log_warn("Ignoring invalid encoder geometry: beltToothSpacing=" << beltToothSpacing << ", encoderTeeth=" << encoderTeeth);
        return;
    }
    _mmPerRevolution = beltToothSpacing * encoderTeeth;
}

double MotorUnit::getMmPerRevolution() const {
    return _mmPerRevolution;
}
