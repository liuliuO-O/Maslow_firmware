"""Production motor output and calibration hold regressions; no hardware motion."""
import importlib.util
import os
from pathlib import Path
import subprocess
import tempfile
import unittest
from test_calibration_bootstrap import compiler_command, extract_function

ROOT = Path(__file__).resolve().parents[3]

class CalibrationHoldControlTest(unittest.TestCase):
    def run_cpp(self, program):
        with tempfile.TemporaryDirectory(prefix='maslow-hold-') as folder:
            cpp = Path(folder) / 'test.cpp'
            exe = Path(folder) / ('test.exe' if os.name == 'nt' else 'test')
            cpp.write_text(program, encoding='utf-8')
            subprocess.run(compiler_command(ROOT) + ['-std=c++17', '-O0', '-I',
                str(ROOT / 'FluidNC/src/Maslow'), str(cpp), '-o', str(exe)], check=True)
            subprocess.run([str(exe)], check=True)

    def test_production_driver_zero_is_no_directional_drive(self):
        source_root=Path(os.environ.get('MASLOW_DRIVER_SOURCE_ROOT',str(ROOT)))
        source = (source_root / 'FluidNC/src/Maslow/DCMotor.cpp').read_text(encoding='utf-8-sig')
        signatures = ['void DCMotor::runAtPWM(', 'void DCMotor::runAtSpeed(', 'void DCMotor::stop(']
        if 'void DCMotor::runAtPWMDirect(' in source:
            signatures.append('void DCMotor::runAtPWMDirect(')
        production = '\n'.join(extract_function(source, sig) for sig in signatures)
        self.run_cpp(r"""
#include <cassert>
#include <cstdint>
#include <cstdlib>
#include <limits>
enum direction { BACKWARD, FORWARD };
int duty[2]={};
void ledcWrite(int channel,int value){assert(value>=0&&value<=1023);duty[channel]=value;}
long map(long x,long a,long b,long c,long d){return (x-a)*(d-c)/(b-a)+c;}
class DCMotor {
public:
 int _maxSpeed=1023,_channel1=0,_channel2=1;
 void runAtPWM(long);void runAtPWMDirect(long);void runAtSpeed(uint8_t,uint16_t);void stop();
};
PRODUCTION
int main(){
 DCMotor m;
 m.runAtPWM(0);assert(duty[0]==1023&&duty[1]==1023);
 for(long n : {1L,75L,512L,1023L}) {
   m.runAtPWM(n);int positive=duty[1]-duty[0];
   m.runAtPWM(-n);assert(positive==duty[0]-duty[1]);
   assert(positive>=75&&positive<=1023);
 }
 m.stop();assert(duty[0]==0&&duty[1]==0);
}
""".replace('PRODUCTION', production).replace('#include <limits>', '#include <limits>\n#include <initializer_list>'))

    def test_continuous_output_damping_slew_limits_and_clock(self):
        self.run_cpp(r"""
#include "CalibrationHoldController.h"
#include <cassert>
#include <cmath>
#include <limits>
using namespace MaslowCalibrationHold;
int main(){
 assert(Controller::driveForEffort(0)==0);
 for(double e=0.001;e<1000;e+=0.25) {
   assert(std::fabs(Controller::driveForEffort(e)+Controller::driveForEffort(-e))<1e-9);
   assert(std::fabs(Controller::driveForEffort(e))<=maxDrivePWM);
 }
 assert(std::fabs(Controller::driveForEffort(.01))<.02);
 assert(std::fabs(Controller::driveForEffort(150)-(150*948.0/1023+75))<1e-9);
 Controller c;assert(c.update(0,-10,0));assert(c.output()==0);
 double last=0;
 uint32_t now=0;
 for(int i=0;i<150;i++) {
   const uint32_t dt=i%2?7:4;now+=dt;
   assert(c.update(now,-10,0));
   assert(std::fabs(c.output()-last)<=maxSlewPWMPerSecond*dt/1000+1e-9);
   assert(std::fabs(c.output())<=maxDrivePWM);last=c.output();
 }
 assert(c.atLimit());
 // Output must cross zero gradually on a reversal, even with a large error.
 bool crossed=false;
 for(int i=0;i<200;i++) {
   now+=4;assert(c.update(now,10,0));
   assert(std::fabs(c.output()-last)<=maxSlewPWMPerSecond*.004+1e-9);
   if(c.output()<=0)crossed=true;last=c.output();
 }
 assert(crossed&&last<0);
 // Same position error: motion toward the target produces less drive than no motion.
 // Keep the error inside the breakaway band so this measures damping, not the assist.
 Controller moving,stationary;
 for(uint32_t t=0;t<=800;t+=4) {
   assert(moving.update(t,-.2,0));assert(stationary.update(t,-.18,0));
 }
 assert(moving.update(804,-.18,0));assert(stationary.update(804,-.18,0));
 assert(moving.velocity()>0&&moving.output()<stationary.output());
 assert(std::fabs(moving.output())<maxDrivePWM-1);
 assert(!moving.update(905,-.98,0));assert(moving.output()==0);
 assert(!moving.update(906,std::numeric_limits<double>::quiet_NaN(),0));
 assert(moving.output()==0);
 // No derivative kick or old drive survives a reference/reset change.
 assert(c.update(now+4,10,5));assert(c.output()==0&&c.velocity()==0);
 c.reset();assert(c.output()==0&&!c.active());
 assert(c.update(0xfffffff0U,-1,0));assert(c.update(0x4U,-1,0));
 assert(c.output()<=maxSlewPWMPerSecond*.020+1e-9);
}
""")

    def test_breakaway_assist_closes_static_friction_offset(self):
        """2026-11-13 wp=1: pure P parked BR at 1.149 mm / 395 PWM and timed out.

        The four measured (error, pwm) pairs all sit on the P curve, so the loop
        was in exact steady state, not saturated.  A plant that only moves above a
        stiction threshold reproduces that deadlock, and the assist must break it
        without leaving a standing force behind.
        """
        self.run_cpp(r"""
#include "CalibrationHoldController.h"
#include <cassert>
#include <cmath>
#include <cstdint>
using namespace MaslowCalibrationHold;

// Belt needs more than 'stick' duty to move; below that it is held by friction.
struct Plant {
    double pos, stick, rate;
    void step(double drive, double dt) {
        if (std::fabs(drive) > stick) pos += (drive < 0 ? -1.0 : 1.0) * rate * dt;
    }
};

// The old law, for the regression that documents the deadlock.
double oldDrive(double kp, double error) {
    return Controller::driveForEffort(kp * error);
}

int main(){
 // 1) Old pure-P law parks at 1.15 mm and never moves again.
 {
   Plant p{1.149, 420.0, 1.5};
   double drive = 0;
   for (int i = 0; i < 2000; ++i) {           // 8 s at 4 ms
     const double error = -p.pos;
     drive = oldDrive(proportionalGain, error);
     p.step(drive, 0.004);
   }
   assert(std::fabs(p.pos) > 1.0);            // still outside the 1 mm gate
   assert(std::fabs(drive - (-394.0)) < 3.0); // parked on the P curve, not at the cap
 }
 // 2) Production controller breaks away and settles inside the gate.
 {
   Plant p{1.149, 420.0, 1.5};
   Controller c;
   assert(c.update(0, p.pos, 0));
   uint32_t now = 0;
   bool inside = false;
   for (int i = 0; i < 2000; ++i) {
     now += 4;
     c.update(now, p.pos, 0);
     p.step(c.output(), 0.004);
     if (std::fabs(p.pos) <= 0.25) inside = true;
   }
   assert(inside);
   assert(std::fabs(p.pos) < 0.35);           // well inside the 1 mm sampling gate
   assert(c.assistOutput() == 0.0);           // released once it could move
   assert(c.stuckMs() == 0);
 }
 // 3) Assist stays idle when the axis is already on target (no creep).
 {
   Controller c;
   c.update(0, 0.05, 0);
   for (uint32_t t = 4; t <= 4000; t += 4) c.update(t, 0.05, 0);
   assert(c.assistOutput() == 0.0);
 }
 // 4) A belt that cannot move at all saturates the cap and reports the stall.
 // The cap is 800 since the 16:40 build, so a 1.149 mm error plus the 380 assist
 // (746 effective) no longer saturates it; the plant starts outside that range.
 {
   Plant p{1.8, 100000.0, 1.5};
   Controller c;
   c.update(0, p.pos, 0);
   for (uint32_t t = 4; t <= 4000; t += 4) c.update(t, p.pos, 0);
   assert(c.assistOutput() >= breakawayLimitPwm - 1.0);
   assert(c.stuckMs() >= breakawayStuckLimitMs);
   assert(c.atLimit());
 }
 // 5) Reset and reference changes clear the learned assist.
 {
   Controller c;
   c.update(0, 5.0, 0);
   for (uint32_t t = 4; t <= 1500; t += 4) c.update(t, 5.0, 0);
   assert(c.assistOutput() > 0.0);
   c.reset();
   assert(c.assistOutput() == 0.0 && c.stuckMs() == 0);
   c.update(2000, 5.0, 0);
   for (uint32_t t = 2004; t <= 3000; t += 4) c.update(t, 5.0, 0);
   assert(c.assistOutput() > 0.0);
   c.update(3004, 5.0, -1.0);                 // new reference
   assert(c.assistOutput() == 0.0);
 }
}
""")

    def test_motor_hold_uses_fresh_feedback_and_resets_on_stop(self):
        source=(ROOT/'FluidNC/src/Maslow/MotorUnit.cpp').read_text(encoding='utf-8-sig')
        production='\n'.join(extract_function(source, sig) for sig in (
            'bool MotorUnit::recomputeCalibrationHold(', 'void MotorUnit::stop(',
            'double MotorUnit::recomputePID(', 'bool MotorUnit::recomputeMotionPID(', 'void MotorUnit::resetPID('))
        self.run_cpp(r"""
#include "CalibrationHoldController.h"
#include <cassert>
#include <cmath>
#include <cstdint>
uint32_t clockMs=0;unsigned long millis(){return clockMs;}
struct Driver {long pwm=0;void stop(){pwm=0;}void runAtPWM(long p){pwm=p;}void runAtPWMDirect(long p){pwm=p;}};
struct PID {double getOutput(double p,double t){return (t-p)*300;}void reset(){}};
class MotorUnit {
public:
 Driver motor;PID positionPID;MaslowCalibrationHold::Controller calibrationHold;
 uint32_t encoderSampleSequence=1,encoderSampleMillis=0,calibrationHoldSequence=0,motionPIDSequence=0;
 double actual=-1,setpoint=0,_commandPWM=0;
 double getPosition(){return actual;}
 bool hasFreshEncoderFeedback(uint32_t now) const { return encoderSampleSequence && now-encoderSampleMillis<=100; }
 bool recomputeCalibrationHold();void stop();double recomputePID();bool recomputeMotionPID();void resetPID();
};
PRODUCTION
int main(){
 MotorUnit a;assert(a.recomputeCalibrationHold());assert(a._commandPWM==0);
 clockMs=4;a.encoderSampleMillis=4;++a.encoderSampleSequence;
 assert(a.recomputeCalibrationHold());assert(a._commandPWM>0&&a._commandPWM<=10);
 const double output=a._commandPWM;
 clockMs=5;a.actual=100; // No new sensor transaction: never differentiate or act on a duplicate sample.
 assert(a.recomputeCalibrationHold());assert(a._commandPWM==output);
 clockMs=105;assert(!a.recomputeCalibrationHold());assert(a.motor.pwm==0&&a._commandPWM==0);
 a.actual=-1;a.encoderSampleMillis=105;++a.encoderSampleSequence;
 assert(a.recomputeCalibrationHold());assert(a._commandPWM==0);
 clockMs=109;a.encoderSampleMillis=109;++a.encoderSampleSequence;assert(a.recomputeCalibrationHold());
 a.stop();assert(a._commandPWM==0&&!a.calibrationHold.active());
 a.recomputePID();assert(a._commandPWM==300&&!a.calibrationHold.active());
 a.resetPID();assert(!a.calibrationHold.active());
 // Proportional motion has a smooth reversal and uses each encoder sample once.
 a.actual=-.01;
 assert(a.recomputeMotionPID());
 const long smallPwm=a.motor.pwm;
 assert(smallPwm>0 && smallPwm<75);
 a.actual=.01;
 assert(a.recomputeMotionPID() && a.motor.pwm==smallPwm);
 ++a.encoderSampleSequence;
 assert(a.recomputeMotionPID() && a.motor.pwm==-smallPwm);
 clockMs+=101;
 assert(!a.recomputeMotionPID() && a.motor.pwm==0);
}
""".replace('PRODUCTION',production))

    def test_capture_guard_recognizes_firmware_sampling_stop(self):
        path=ROOT/'.pio/diagnostics/calibration_capture_guard.py'
        spec=importlib.util.spec_from_file_location('hold_guard_regression',path)
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        guard=module.CalibrationGuard();guard.start(0)
        guard.observe('[MSG:ERR: Horizontal bootstrap measurement stopped: no stable sampling window within 8 seconds wp=1]',1)
        self.assertFalse(guard.active)
        self.assertIsNone(guard.check(10))
        guard.start(0)
        self.assertIsNotNone(guard.check(6))

if __name__=='__main__':unittest.main()
