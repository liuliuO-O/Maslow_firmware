// Copyright (c) 2024 Maslow CNC. All rights reserved.
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file with
// following exception: it may not be used for any reason by MakerMade or anyone with a business or personal connection to MakerMade

#ifndef MotorUnit_h
#define MotorUnit_h

#include "Arduino.h"
#include "Wire.h"
#include "AS5600.h"
#include "CalibrationHoldController.h"
#include "MiniPID.h"  //https://github.com/tekdemo/MiniPID
#include "DCMotor.h"
#include "memory"
#include "SparkFun_I2C_Mux_Arduino_Library.h"

class MotorUnit {
public:
    void     begin(int forwardPin, int backwardPin, int readbackPin, int encoderAddress, int channel1, int channel2,
                   bool reverseEncoder = false);
    void     zero();
    void     setTarget(double newTarget);
    double   getTarget();
    double   getPosition();
    void     setPosition(double position);  // Set encoder position from saved value
    uint16_t getRawEncoderAngle();          // Get current raw encoder angle (0-4095)
    // Set belt geometry used for encoder counts <-> mm conversion.
    void     setEncoderGeometry(double beltToothSpacing, double encoderTeeth);
    // Returns the configured belt movement in mm per full encoder revolution.
    double   getMmPerRevolution() const;
    double   getCurrent();
    double   getPositionError();
    void     stop();
    void     driveIn(int pwm);
    bool     updateEncoderPosition();
    uint32_t getEncoderReadFailures() const { return encoderReadFailures; }
    // Lifetime counter; observing it does not filter or modify cumulative position.
    uint32_t getEncoderLargeSteps() const { return encoderLargeSteps; }
    bool hasFreshEncoderFeedback(uint32_t now) const {
        return encoderSampleSequence != 0 && now - encoderSampleMillis <= MaslowCalibrationHold::maxFeedbackGapMs;
    }
    double   recomputePID();
    bool     recomputeMotionPID();
    bool     recomputeCalibrationHold();
    double   getCalibrationHoldVelocity() const { return calibrationHold.velocity(); }
    void     decompressBelt();
    void     driveOut(int speed);  // Drive motor outward at specified PWM speed (0-1023)
    bool     comply();
    bool     retract();
    bool     extend(double targetLength);
    bool     pull_tight(int currentThreshold);
    bool     motor_test();
    void     fullOut();
    void     fullIn();
    bool     test();
    void     reset();     //resetting variables here, because of non-blocking, maybe there's a better way to do this
    void     resetPID();  // Reset PID state (clears integral accumulator) to prevent I-term residual motion after stop

    double getMotorCurrent();  //averaged value of the last 10 measurements
    double getBeltSpeed();
    double getMotorPower();
    void   update();
    bool   onTarget(double precision);
    // Lifetime count of latched "commanded but no current and no motion" events.
    // Diagnostic only: never alters motor output or calibration state.
    uint32_t getDriveLossEvents() const { return driveLossEvents; }
    bool     isDriveLossActive() const { return driveLossActive; }

private:
    MaslowCalibrationHold::Controller calibrationHold;
    uint32_t encoderSampleSequence = 0;
    uint32_t encoderSampleMillis = 0;
    uint32_t calibrationHoldSequence = 0;
    uint32_t motionPIDSequence = 0;
    uint32_t encoderLargeSteps = 0;
    uint32_t encoderReadFailures = 0;  // Cumulative since boot, including mux selection failures.
    bool     encoderDiagnosticInitialized = false;
    int32_t  encoderDiagnosticLastCounts = 0;
    unsigned long encoderDiagnosticLastTime = 0;
    unsigned long encoderDiagnosticLastPrintTime = 0;
    uint32_t encoderDiagnosticSuppressed = 0;
    int     _encoderAddress;
    AS5600  encoder;
    MiniPID positionPID;  //These are the P,I,D values for the servo motors
    DCMotor motor;
    double  setpoint                           = 0.0;
    double  _mmPerRevolution                   = 43.975;  //If the amount of belt extended is too long, this number needs to be bigger
    int     _stallThreshold                    = 25;      //The number of times in a row needed to trigger a warning
    int     _stallCurrent                      = 27;      //The current threshold needed to count
    int     _stallCount                        = 0;
    int     _numPosErrors                      = 0;  //Keeps track of the number of position errors in a row to detect a stall
    double  _lastPosition                      = 0.0;
    double  _commandPWM                        = 0;  //The last PWM duty cycle sent to the motor
    double  mostRecentCumulativeEncoderReading = 0;
    double  encoderReadFailurePrintTime        = millis();
    //unsigned long lastCallGetPos = millis();

    //variables to keep track of the motor current and belt speed
    double        beltSpeed             = 0;
    unsigned long beltSpeedTimer        = millis();
    double        beltSpeedLastPosition = 0;
    double        motorCurrentBuffer[10];
    unsigned long motorCurrentTimer = millis();

    //These are used when retracting the belts for storage
    int      absoluteCurrentThreshold = 1300;
    int      incrementalThreshold     = 125;
    int      incrementalThresholdHits = 0;
    float    alpha                    = .2;
    uint16_t retract_speed            = 0;
    float    retract_baseline         = 700;

    //comply variables
    unsigned long lastCallToComply  = millis();
    unsigned long lastCallToRetract = millis();
    double        lastPosition      = getPosition();
    double        amtToMove         = 0.1;

    int beltSpeedCounter = 0;

    // Drive-loss capture state (DRV8873 protection / supply dropout signature:
    // PWM commanded, current reads ~0, belt does not move).  Latches the event
    // and logs a snapshot so the failure can be inspected BEFORE any power
    // swap clears the driver fault.  No control authority.
    unsigned long driveLossCheckMs  = 0;
    unsigned long driveLossRunMs    = 0;
    double        driveLossLastPos  = 0;
    uint32_t      driveLossEvents   = 0;
    bool          driveLossActive   = false;
    bool          driveLossPosInit  = false;
};

#endif
