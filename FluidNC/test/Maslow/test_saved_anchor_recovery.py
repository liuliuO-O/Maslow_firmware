"""Exercise the production saved-anchor restore and fit with simulated I/O."""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest
from test_calibration_bootstrap import compiler_command, extract_function

ROOT = Path(__file__).resolve().parents[3]


class SavedAnchorRecoveryTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        calibration = (ROOT / 'FluidNC/src/Maslow/Calibration.cpp').read_text(encoding='utf-8-sig')
        machine = (ROOT / 'FluidNC/src/Maslow/Maslow.cpp').read_text(encoding='utf-8-sig')
        production = '\n'.join(extract_function(calibration, signature) for signature in (
            'bool Calibration::beginApplyTension(',
            'bool Calibration::failApplyTension(',
            'bool Calibration::applyTensionFeedbackHealthy(',
            'bool Calibration::updateApplyTensionHold(',
            'bool Calibration::savedAnchorHoldSettled(',
            'bool Calibration::keepSettledSavedAnchorHold(',
            'void Calibration::logSavedAnchorSettling(',
            'void Calibration::logSavedAnchorSampling(',
            'bool Calibration::beginSavedAnchorMotionSettling(',
            'bool Calibration::maintainSavedAnchorHold(',
            'bool Calibration::takeSlackFunc(',
            'bool Calibration::all_axis_homed(',
            'bool Calibration::allAxisExtended(',
        )) + '\n' + extract_function(machine, 'void Maslow_::stopMotors(')
        geometry = (ROOT / 'FluidNC/src/Kinematics/MaslowKinematics.cpp').read_text(encoding='utf-8-sig')
        production += '\n' + extract_function(geometry, 'double MaslowKinematics::compute(').replace('MaslowKinematics::', 'FieldGeometry::')
        cls.temp = tempfile.TemporaryDirectory(prefix='maslow-saved-anchor-')
        cls.addClassCleanup(cls.temp.cleanup)
        folder = Path(cls.temp.name)
        (folder / 'Arduino.h').write_text('#pragma once\n#include <cstdint>\nunsigned long millis();\n', encoding='utf-8')
        cpp = folder / 'recovery.cpp'
        cls.binary = folder / ('recovery.exe' if os.name == 'nt' else 'recovery')
        cpp.write_text(HARNESS.replace('PRODUCTION_CODE', production), encoding='utf-8')
        subprocess.run(compiler_command(ROOT) + ['-std=c++17', '-O0',
            '-I', str(folder), '-I', str(ROOT / 'FluidNC/src/Maslow'),
            str(cpp), '-o', str(cls.binary)], check=True)

    def check(self, scenario):
        subprocess.run([str(self.binary), scenario], check=True)

    def test_coordinated_preload_then_off_center_ready_hold(self):
        self.check('success')

    def test_preload_stops_when_a_belt_has_no_current_or_inward_progress(self):
        self.check('no_response')

    def test_drift_feedback_time_and_retraction_fail_safely(self):
        self.check('faults')

    def test_geometry_gates_all_four_belts_without_refitting_anchors(self):
        self.check('geometry')

    def test_failed_restore_can_restart_without_repeating_anchor_calibration(self):
        self.check('retry')

    def test_motion_settles_before_ready_hold_and_save(self):
        self.check('settle')


HARNESS = r'''
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
#include "CalibrationSampleWindow.h"
#include "SavedAnchorRecovery.h"

constexpr int Z_AXIS = 2;
uint32_t clockMs = 0;
unsigned long millis() { return clockMs; }
std::vector<std::string> logs;
#define log_info(x) do {std::ostringstream s;s << x;logs.push_back(s.str());} while(false)
#define log_warn(x) log_info(x)
#define log_error(x) log_info(x)
#define log_debug(x) log_info(x)
void serviceCalibrationWatchdogs(bool=false) {}
enum class State { Idle, Homing, Alarm };
struct { State value=State::Idle; void set_state(State s){value=s;} State state() const {return value;} } sys;
struct Axis {
    double position=0, target=0, settledLength=0, pwm=0, velocity=0;
    bool fresh=true, holdHealthy=true, noCurrent=false;
    int pulls=0, holds=0, finishAfter=1, targets=0;
    uint32_t failures=0, steps=0;
    double getPosition() const { return position; }
    double getTarget() const { return target; }
    double getPositionError() const { return position-target; }
    double getMotorPower() const { return pwm; }
    bool hasFreshEncoderFeedback(uint32_t) const { return fresh; }
    uint32_t getEncoderReadFailures() const { return failures; }
    uint32_t getEncoderLargeSteps() const { return steps; }
    void stop() { pwm=0; }
    void reset() { pulls=0; }
    void resetPID() {}
    void setTarget(double p) { target=p; ++targets; }
    double getMotorCurrent() const { return noCurrent ? 0 : pulls>=finishAfter && pwm<0 ? 765 : 250; }
    double getBeltSpeed() const { return 0; }
    double getCalibrationHoldVelocity() const { return velocity; }
    void driveIn(int duty) {
        ++pulls;
        pwm=-duty;
        if (pulls>=finishAfter) position=settledLength;
    }
    bool recomputeCalibrationHold() {
        ++holds;
        pwm=std::clamp(300*(target-position),-600.,600.);
        return holdHealthy;
    }
};
struct Maslow_ {
    Calibration calibration;
    Axis axis[4];
    float scaleX=1, scaleY=1;
    float workAreaX=500, workAreaY=300, workAreaCenterOffsetX=0, workAreaCenterOffsetY=0;
    int motionUpdates=0, positionSaves=0;
    void stopMotors();
    void recomputePID() { ++motionUpdates; }
    void saveZPos() {}
    void saveBeltPositions() { ++positionSaves; }
    void markBeltPositionsStale() {}
} Maslow;
struct Kinematics {
    float getCenterX() const { return 493; }
    float getCenterY() const { return 536; }
    double compute(int arm,double x,double y,double z) const {
        static constexpr double ax[4]={-490,499,-489,489};
        static constexpr double ay[4]={540,535,-540,-540};
        static constexpr double az[4]={-40,20,-20,-10};
        return std::hypot(std::hypot(ax[arm]-x,ay[arm]-y)-153.4,z+az[arm]);
    }
} kin;
Kinematics* getKinematics() {return &kin;}

struct FieldGeometry {
    double compute(int arm, double x, double y, double z) const;
    const float anchor_location[ARM_COUNT][3] = {
        {-1.432132f,1080.465942f,-40}, {989.286133f,1074.734497f,20}, {0,0,-20}, {979.778625f,0,-10}
    };
    float _centerX=493.1997024f, _centerY=535.7992156f;
    float _beltEndExtension=30, _armLength=123.400002f, _spoilboardThickness=0, _workThickness=0;
    bool _fixedZ=false;
};
float mpos[5]={0,0,3,0,0};
float* get_mpos() {return mpos;}
int readyTransitions=0;
Calibration::Calibration() {}
void Calibration::printCurrentState() {}
bool Calibration::requestStateChange(int state) {
    assert(state==READY_TO_CUT && currentState==TAKING_SLACK);
    ++readyTransitions; currentState=state; sys.set_state(State::Idle);
    return true;
}
PRODUCTION_CODE

void prepare(double x=12, double y=-8, double bias=0) {
    clockMs=10;
    Maslow=Maslow_{};
    auto& c=Maslow.calibration;
    c.currentState=EXTENDEDOUT;
    c.retractCurrentThreshold=850;
    sys.set_state(State::Idle);
    readyTransitions=0;
    logs.clear();
    for (int a=0;a<4;++a) {
        auto& axis=Maslow.axis[a];
        c.axis_homed[a]=c.extended[a]=true;
        axis.finishAfter=a+1;
        axis.settledLength=kin.compute(a,x,y,mpos[2])+(a==0?bias:0);
        axis.position=axis.settledLength+20;
    }
}
void begin() {
    auto& c=Maslow.calibration;
    assert(c.beginApplyTension());
    c.currentState=TAKING_SLACK;
    sys.set_state(State::Homing);
}
bool tick(uint32_t elapsed=25) {
    clockMs+=elapsed; return Maslow.calibration.takeSlackFunc();
}
void prime() {
    for (int i=0;i<100 && !Maslow.calibration.applyTensionRecovery.sampling;++i) assert(!tick());
    assert(Maslow.calibration.applyTensionRecovery.sampling);
}
void finish() {
    bool done=false;
    for (int i=0;i<600 && !done;++i) done=tick();
    assert(done);
}
void stopped(int expectedState, bool homed) {
    auto& c=Maslow.calibration;
    assert(c.currentState==expectedState && c.all_axis_homed()==homed);
    assert(sys.value==State::Alarm && readyTransitions==0);
    assert(!c.applyTensionRecovery.active && !c.savedAnchorHoldActive);
    for (const auto& a:Maslow.axis) assert(a.pwm==0);
}
int main(int argc,char**argv) {
    assert(argc==2);
    const std::string scenario=argv[1];
    auto& c=Maslow.calibration;
    if (scenario=="success") {
        prepare(173,-107); begin();
        // All outputs start together, and an early-loaded belt keeps applying
        // load while the last belt is still taking up slack.
        assert(!tick());
        for (const auto& axis : Maslow.axis) assert(axis.pulls==1 && axis.pwm<0);
        assert(!tick());
        assert(Maslow.axis[0].pulls==2 && Maslow.axis[0].pwm<0);
        assert(Maslow.axis[0].holds==0 && Maslow.axis[0].targets==0);
        prime();
        for (const auto& axis : Maslow.axis) {
            assert(axis.targets==1 && std::fabs(axis.target-axis.position)<.001);
        }
        finish();
        assert(readyTransitions==1 && c.currentState==READY_TO_CUT && sys.value==State::Idle);
        assert(c.applyTensionRecovery.snapshots==6 && c.savedAnchorHoldActive);
        assert(clockMs<6000);
        bool recoveredOffCenter=false;
        for (const auto& message : logs) {
            if (message.find("RESTORE fit converged=1 x=173 y=-107")!=std::string::npos) recoveredOffCenter=true;
        }
        assert(recoveredOffCenter);
        for (const auto& axis : Maslow.axis) assert(std::fabs(axis.getTarget()-axis.getPosition())<.001);
        assert(c.maintainSavedAnchorHold());
        assert(c.keepSettledSavedAnchorHold());
        Maslow.stopMotors(); // Production stop must disarm the Idle hold.
        assert(!c.savedAnchorHoldActive && !c.maintainSavedAnchorHold());
        for (const auto& a:Maslow.axis) assert(a.pwm==0);
    } else if (scenario=="settle") {
        prepare(); begin(); finish();
        assert(c.beginSavedAnchorMotionSettling());
        Maslow.axis[0].position += 8;
        const double target=Maslow.axis[0].getTarget();
        assert(c.maintainSavedAnchorHold());
        assert(Maslow.motionUpdates==1 && Maslow.positionSaves==0);
        Maslow.axis[0].position=target;
        assert(c.maintainSavedAnchorHold());
        assert(!c.keepSettledSavedAnchorHold());
        clockMs+=100;
        assert(c.maintainSavedAnchorHold());
        assert(Maslow.positionSaves==1 && !c.applyTensionRecovery.motionSettling);
        assert(Maslow.axis[0].getTarget()==target);
        assert(c.beginSavedAnchorMotionSettling());
        Maslow.axis[0].position+=8;
        clockMs+=1450;
        assert(c.maintainSavedAnchorHold());
        Maslow.axis[0].position=target+.25;
        assert(c.maintainSavedAnchorHold());
        clockMs+=50;
        assert(c.maintainSavedAnchorHold());
        assert(std::any_of(logs.begin(), logs.end(), [](const auto& line) {
            return line.find("MOTION_SETTLE result=stopped elapsed_ms=1500 longest_in_tolerance_ms=50")!=std::string::npos
                && line.find("outside_mask=1 peak_error_mm TL=8")!=std::string::npos;
        }));
        assert(sys.value==State::Alarm && c.currentState==EXTENDEDOUT && c.all_axis_homed());
        assert(Maslow.positionSaves==1);
    } else if (scenario=="no_response") {
        prepare();
        auto& br = Maslow.axis[_BR];
        br.finishAfter=10000;
        br.noCurrent=true;
        begin();
        finish();
        stopped(EXTENDEDOUT,true);
        assert(clockMs>=5700 && clockMs<6500);
        assert(std::any_of(logs.begin(),logs.end(),[](const auto& line) {
            return line.find("RESTORE_NO_RESPONSE belt=BR")!=std::string::npos;
        }));

        MaslowSavedAnchorRecovery::PreloadDrive monitor;
        assert(!monitor.unresponsive(0,899,0,716.536));
        assert(!monitor.unresponsive(10,1023,0,716.536));
        assert(!monitor.unresponsive(510,1023,0,715.936)); // A freely retracting belt can read zero ADC.
        assert(!monitor.unresponsive(1010,1023,0,715.336));
        assert(!monitor.unresponsive(1250,1023,15,715.336)); // Measured current resets the window.
        assert(!monitor.unresponsive(1260,1023,0,715.336));
        assert(!monitor.unresponsive(2009,1023,0,715.336));
        assert(monitor.unresponsive(2010,1023,0,715.336));
    } else if (scenario=="faults") {
        prepare();c.axis_homed[0]=false;assert(!c.beginApplyTension());
        for (const auto& a:Maslow.axis) assert(a.pulls==0);

        prepare();begin();++Maslow.axis[1].steps;assert(tick());stopped(UNKNOWN,false);
        for (const auto& a:Maslow.axis) assert(a.pulls==0);
        prepare();begin();prime();++Maslow.axis[2].failures;assert(tick());stopped(UNKNOWN,false);
        prepare();begin();prime();Maslow.axis[3].fresh=false;assert(tick());stopped(UNKNOWN,false);
        prepare();begin();Maslow.axis[0].position-=301;assert(tick());stopped(EXTENDEDOUT,true);
        prepare();begin();assert(tick(30000));stopped(EXTENDEDOUT,true);
        prepare();begin();assert(tick(60000));stopped(EXTENDEDOUT,true);

        prepare();begin();prime();
        int preloadUpdates=Maslow.axis[0].pulls;
        bool done=false;
        for (int i=0;i<500 && !done;++i) {
            // Position remains inside the 1 mm target gate, but keeps oscillating.
            Maslow.axis[0].position=Maslow.axis[0].target+(i%2?0.75:-0.75);
            done=tick();
        }
        assert(done);stopped(EXTENDEDOUT,true);
        assert(std::any_of(logs.begin(), logs.end(), [](const auto& line) {
            return line.find("RESTORE_SAMPLE result=stopped snapshots=0 position_error_mask=0 pwm_limit_mask=0 span_reset_mask=1")!=std::string::npos;
        }));
        for (const auto& axis:Maslow.axis) assert(axis.pulls==preloadUpdates);

        prepare();begin();prime();Maslow.axis[0].position+=8;
        assert(!tick());assert(tick(1500));stopped(EXTENDEDOUT,true);
    } else if (scenario=="geometry") {
        // Four lengths within the failed log's printed rounding, including the low-residual case.
        const FieldGeometry field;
        const double fieldLengths[ARM_COUNT]={583.50935,593.03165,559.93335,570.77565};
        const auto fieldFit=MaslowSavedAnchorRecovery::fitPosition(fieldLengths,
            [&field](int arm,double x,double y){return field.compute(arm,x,y,0);}, [](){});
        assert(fieldFit.acceptable(2.5) && fieldFit.maxResidual<0.85);
        assert(std::hypot(fieldFit.x+10.916,fieldFit.y+9.559)<.02);

        MaslowSavedAnchorRecovery::PreloadDrive preload;
        for (int i=0;i<100;++i) preload.update(.01,250,0,850,8);
        const double duty=preload.duty;
        preload.update(.01,250,-15,850,8);
        assert(preload.duty<duty);
        preload.update(.01,1000,0,850,8);
        assert(preload.duty<duty);
        assert(!MaslowSavedAnchorRecovery::PreloadDrive::loaded(765,-8,850));
        assert(MaslowSavedAnchorRecovery::PreloadDrive::loaded(765,0,850));
        const auto predict=[](int arm,double x,double y){return kin.compute(arm,x,y,mpos[2]);};
        double lengths[4];
        for (int arm=0;arm<4;++arm) lengths[arm]=predict(arm,46,-31);
        auto fit=MaslowSavedAnchorRecovery::fitPosition(lengths,predict,[](){});
        assert(fit.acceptable(2.5) && std::hypot(fit.x-46,fit.y+31)<.01);
        lengths[0]+=6;
        fit=MaslowSavedAnchorRecovery::fitPosition(lengths,predict,[](){});
        assert(fit.converged && !fit.acceptable(2.5) && fit.acceptable(5));
        lengths[0]=std::numeric_limits<double>::quiet_NaN();
        assert(!MaslowSavedAnchorRecovery::fitPosition(lengths,predict,[](){}).converged);
        for (double& length:lengths) length=500;
        assert(!MaslowSavedAnchorRecovery::fitPosition(lengths,[](int,double,double){return 500.;},[](){}).converged);

        prepare(12,-8,6);begin();finish();stopped(EXTENDEDOUT,true);
        prepare(12,-8,6);c.applyTensionMaxResidualMm=5;begin();finish();assert(readyTransitions==1);
        prepare(300,0);begin();finish();stopped(EXTENDEDOUT,true);
    } else if (scenario=="retry") {
        prepare();begin();prime();assert(tick(12000));stopped(EXTENDEDOUT,true);
        // Acknowledging a quality failure needs no new home or anchor fit.
        sys.set_state(State::Idle);begin();finish();
        assert(readyTransitions==1 && c.all_axis_homed());
        ++Maslow.axis[0].failures;
        assert(c.maintainSavedAnchorHold());
        assert(c.currentState==UNKNOWN && sys.value==State::Alarm && !c.all_axis_homed());
    } else assert(false);
}
'''

if __name__ == '__main__':
    unittest.main()
