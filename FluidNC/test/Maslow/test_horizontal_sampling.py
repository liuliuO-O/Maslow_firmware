"""Exercise fixed-reference bootstrap sampling with production C++ and simulated I/O."""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest
from test_calibration_bootstrap import compiler_command, extract_function

class HorizontalSamplingTest(unittest.TestCase):
    def compile_and_run(self, program):
        root=Path(__file__).resolve().parents[3]
        with tempfile.TemporaryDirectory(prefix="maslow-horizontal-sampling-") as directory:
            folder=Path(directory)
            (folder/"Arduino.h").write_text("#pragma once\n#include <cstdint>\nunsigned long millis();\n",encoding="utf-8")
            cpp=folder/"test.cpp"
            binary=folder/("test.exe" if os.name=="nt" else "test")
            cpp.write_text(program,encoding="utf-8")
            subprocess.run(compiler_command(root)+["-std=c++17","-O0","-I",str(folder),
                "-I",str(root/"FluidNC/src/Maslow"),str(cpp),"-o",str(binary)],check=True)
            subprocess.run([str(binary)],check=True)

    def test_stability_rejects_drift_gaps_invalid_feedback_and_handles_clock_wrap(self):
        self.compile_and_run(r"""
#include "CalibrationStabilityWindow.h"
#include <cassert>
#include <limits>
using namespace MaslowCalibrationStability;
int main() {
    Window window;
    float p[4]={100,200,300,400};
    for(uint32_t t=0;t<750;t+=25) assert(!window.update(t,p,true));
    assert(window.update(750,p,true));
    window.reset();
    // Small adjacent changes still accumulate into disallowed drift.
    for(uint32_t t=0;t<=2000;t+=25) {
        p[0]=100+0.001f*t;
        assert(!window.update(t,p,true));
    }
    p[0]=100;window.reset();
    assert(!window.update(0,p,true));
    assert(!window.update(750,p,true)); // Two isolated observations are insufficient.
    assert(!window.update(775,p,false));
    p[1]=std::numeric_limits<float>::quiet_NaN();
    assert(!window.update(800,p,true));
    p[1]=200;
    for(uint32_t t=825;t<1575;t+=25) assert(!window.update(t,p,true));
    assert(window.update(1575,p,true));
    // Firmware millis() wrap must not shorten or stall the window.
    window.reset();
    const uint32_t begin=0xfffffff0U;
    for(uint32_t dt=0;dt<750;dt+=25) assert(!window.update(begin+dt,p,true));
    assert(window.update(begin+750,p,true));
}
""")

    def test_production_fixed_reference_scope_holding_timeouts_and_feedback_faults(self):
        root=Path(__file__).resolve().parents[3]
        source=(root/"FluidNC/src/Maslow/Calibration.cpp").read_text(encoding="utf-8-sig")
        production="\n".join(extract_function(source,signature) for signature in (
            "void Calibration::resetHorizontalSampling(",
            "bool Calibration::failHorizontalSampling(",
            "bool Calibration::takeStableHorizontalMeasurement(",
            "bool Calibration::take_measurement(",
            "void Calibration::resetMeasurementState(",
        ))
        self.compile_and_run(r"""
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <limits>
#include <sstream>
#include <string>
#include <vector>
#define private public
#include "Calibration.h"
#undef private
unsigned long clockMs=100;
unsigned long millis(){return clockMs;}
using std::abs;
using std::min;
using std::max;
std::vector<std::string> logs;
#define log_info(x) do {std::ostringstream s;s<<x;logs.push_back(s.str());}while(false)
#define log_error(x) log_info(x)
void serviceCalibrationWatchdogs(bool=false){}
struct Axis {
    double position=1000,target=1000,pwm=0;
    int pulls=0,pids=0,sets=0,stops=0,finishAfter=1;
    uint32_t failures=0,steps=0;
    bool holdHealthy=true;
    double getPosition(){return position;}
    double getTarget(){return target;}
    double getMotorPower(){return pwm;}
    uint32_t getEncoderReadFailures(){return failures;}
    uint32_t getEncoderLargeSteps(){return steps;}
    void setTarget(double p){target=p;++sets;}
    void stop(){pwm=0;++stops;}
    void reset(){}
    void resetPID(){}
    double recomputePID(){++pids;pwm=std::max(-1023.0,std::min(1023.0,(target-position)*300));return pwm;}
    bool recomputeCalibrationHold(){if(!holdHealthy){pwm=0;return false;}recomputePID();return true;}
    bool pull_tight(int){++pulls;pwm=-500;if(pulls>=finishAfter){stop();return true;}return false;}
};
using MotorUnit = Axis;
struct Machine {
    Axis axis[4];float x=0,y=0;int stops=0;
    void stopMotors(){++stops;for(auto& a:axis)a.stop();}
} Maslow;
struct Kinematics {
    float getTlZ(){return 0;}float getTrZ(){return 0;}
    float getBlZ(){return 0;}float getBrZ(){return 0;}
    float getSpoilboardThickness(){return 0;}float getWorkThickness(){return 0;}
} kin;
bool haveKinematics=true;
Kinematics* getKinematics(){return haveKinematics?&kin:nullptr;}
float mpos[5]={};
float* get_mpos(){return mpos;}
Calibration::Calibration(){}
void Calibration::diagnosticStage(const char* stage){diagnosticPhase=stage;}
float Calibration::measurementToXYPlane(float length,float){return length;}
PRODUCTION
void setup(Calibration& c) {
    clockMs=100;Maslow=Machine{};haveKinematics=true;
    c.calibrationInProgress=true;
    for(int i=0;i<4;i++){Maslow.axis[i].position=1000+10*i;Maslow.axis[i].target=-1;c.axis_homed[i]=true;}
}
bool step(Calibration& c,float out[4],int point=0,unsigned long ms=25) {
    clockMs+=ms;return c.take_measurement(out,UP,0,850,point);
}
void prime(Calibration& c,float out[4]) {
    assert(!step(c,out)); // One preload.
    assert(c.horizontalInitialTensionComplete);
    assert(!step(c,out)); // First bottom belt reaches tension.
    assert(!step(c,out)); // Second bottom belt reaches tension.
    assert(c.horizontalSampling.holdingSamples);
}
bool snapshot(Calibration& c,float out[4]) {
    for(int i=0;i<35;i++)if(step(c,out))return true;
    return false;
}
int main() {
    float out[4]={};
    Calibration c;setup(c);prime(c,out);
    int pulls[4],sets[4];double targets[4];
    for(int i=0;i<4;i++){pulls[i]=Maslow.axis[i].pulls;sets[i]=Maslow.axis[i].sets;targets[i]=Maslow.axis[i].target;}
    // Warm-ups and retained samples observe the same references and preload.
    for(int sample=0;sample<6;sample++){
        assert(snapshot(c,out));
        for(int i=0;i<4;i++){
            assert(out[i]==1000+10*i);
            assert(Maslow.axis[i].pulls==pulls[i]);
            assert(Maslow.axis[i].sets==sets[i] && Maslow.axis[i].target==targets[i]);
        }
    }
    assert(!step(c,out,1)); // New waypoint gets its own preload/reference.
    assert(c.horizontalSampling.point==1 && Maslow.axis[_TL].pulls==pulls[_TL]+1);
    c.resetMeasurementState();
    assert(c.horizontalSampling.point==-1 && !c.horizontalInitialTensionComplete);

    // The first completed bottom axis actively holds while the other is pending.
    Calibration staged;setup(staged);assert(!step(staged,out));
    Maslow.axis[_BL].finishAfter=4;assert(!step(staged,out));
    const int firstPulls=Maslow.axis[_BR].pulls;
    for(int i=0;i<3;i++)assert(!step(staged,out));
    assert(Maslow.axis[_BR].pids>=3 && Maslow.axis[_BR].pulls==firstPulls);

    // A discontinuity after reference capture stops before any further PID use.
    Calibration jump;setup(jump);prime(jump,out);
    const int pids=Maslow.axis[_TL].pids;++Maslow.axis[_TL].steps;
    assert(!step(jump,out));
    assert(jump.horizontalMeasurementFault && Maslow.stops==1);
    assert(jump.horizontalSamplingReferenceInvalid);
    for(int i=0;i<4;i++)assert(!jump.axis_homed[i]);
    assert(Maslow.axis[_TL].pids==pids && jump.horizontalSampling.point==-1);
    Calibration readFailure;setup(readFailure);prime(readFailure,out);++Maslow.axis[_TR].failures;
    assert(!step(readFailure,out) && readFailure.horizontalMeasurementFault);

    Calibration stale;setup(stale);prime(stale,out);Maslow.axis[_TL].holdHealthy=false;
    assert(!step(stale,out) && stale.horizontalMeasurementFault);
    for(int i=0;i<4;i++)assert(!stale.axis_homed[i]);

    // Invalid feedback never starts a pull. Settling cannot run indefinitely.
    Calibration invalid;setup(invalid);Maslow.axis[0].position=std::numeric_limits<double>::quiet_NaN();
    assert(!step(invalid,out) && invalid.horizontalMeasurementFault);
    for(auto& a:Maslow.axis)assert(a.pulls==0);
    Calibration drifting;setup(drifting);prime(drifting,out);Maslow.axis[_TL].position+=2;
    assert(!step(drifting,out,0,8000) && drifting.horizontalMeasurementFault);
    Calibration preloadTimeout;setup(preloadTimeout);Maslow.axis[_BL].finishAfter=100000;
    assert(!step(preloadTimeout,out));
    assert(!step(preloadTimeout,out,0,30000) && preloadTimeout.horizontalMeasurementFault);
    Calibration bottomTimeout;setup(bottomTimeout);
    assert(!step(bottomTimeout,out));Maslow.axis[_BR].finishAfter=100000;
    assert(!step(bottomTimeout,out,0,30000) && bottomTimeout.horizontalMeasurementFault);
    Calibration conflict;setup(conflict);prime(conflict,out);Maslow.axis[_TL].position+=50;
    assert(!step(conflict,out));assert(!step(conflict,out,0,1500));
    assert(conflict.horizontalMeasurementFault);

    // The new policy is limited to active horizontal calibration's first six points.
    Calibration tension;setup(tension);tension.calibrationInProgress=false;
    assert(!step(tension,out));assert(tension.horizontalSampling.point==-1);
    Calibration vertical;setup(vertical);vertical.orientation=VERTICAL;
    step(vertical,out);assert(vertical.horizontalSampling.point==-1);
    Calibration grid;setup(grid);step(grid,out,6);assert(grid.horizontalSampling.point==-1);
}
""".replace("PRODUCTION",production))

if __name__=="__main__":
    unittest.main()
