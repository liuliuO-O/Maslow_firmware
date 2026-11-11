"""Compile and exercise the production calibration sample-window policy."""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest
from test_calibration_bootstrap import compiler_command, extract_function


class CalibrationSampleWindowTest(unittest.TestCase):
    def test_drift_outlier_rollover_and_invalid_samples(self):
        root = Path(__file__).resolve().parents[3]
        program = r"""
#include "CalibrationSampleWindow.h"
#include <cassert>
#include <cmath>
#include <limits>
using namespace MaslowCalibrationSamples;
int main() {
    float drift[4][4] = {{100,200,300,400},{110,200,300,400},
                         {120,200,300,400},{130,200,300,400}};
    auto q = evaluate(drift);
    assert(q.adjacent == 10 && q.span == 30 && q.arm == 0);
    assert(q.minimumSample == 0 && q.maximumSample == 3);
    // The old adjacent-only 15 mm gate would have accepted this 30 mm drift.
    assert(q.adjacent <= 15 && q.span > 15);

    float actual[4][4] = {{816.888f,544.142f,924.708f,664.044f},
                          {803.562f,541.938f,924.278f,675.802f},
                          {781.554f,541.186f,923.323f,687.302f},
                          {777.455f,540.745f,922.732f,698.888f}};
    q = evaluate(actual);
    assert(q.arm == 0 && std::fabs(q.span - 39.433f) < .001f);
    assert(std::fabs(q.adjacent - 22.008f) < .001f);

    float snapshots[4][4] = {{80,200,300,400},{100,201,301,401},
                             {101,202,302,402},{102,203,303,403}};
    assert(evaluate(snapshots).span > 15);
    slide(snapshots);
    // All four columns shift together, preserving simultaneous belt snapshots.
    assert(snapshots[0][0] == 100 && snapshots[0][3] == 401);
    for (int arm = 0; arm < 4; ++arm) snapshots[3][arm] = 103 + 100 * arm;
    assert(evaluate(snapshots).span == 3);
    // Do not accept while an interior outlier remains in the window.
    snapshots[1][2] = 350;
    assert(evaluate(snapshots).span == 49 && evaluate(snapshots).arm == 2);

    float boundary[4][4] = {{100,200,300,400},{105,205,305,405},
                            {110,210,310,410},{115,215,315,415}};
    assert(evaluate(boundary).span == 15);
    boundary[3][3] += .01f;
    assert(evaluate(boundary).span > 15 && evaluate(boundary).arm == 3);
    boundary[0][1] = std::numeric_limits<float>::quiet_NaN();
    assert(!std::isfinite(evaluate(boundary).span));
    boundary[0][1] = 0;
    assert(!std::isfinite(evaluate(boundary).span));
}
"""
        with tempfile.TemporaryDirectory(prefix="maslow-window-") as directory:
            cpp = Path(directory) / "window.cpp"
            binary = Path(directory) / ("window.exe" if os.name == "nt" else "window")
            cpp.write_text(program, encoding="utf-8")
            subprocess.run(compiler_command(root) + ["-std=c++11", str(cpp),
                "-I", str(root / "FluidNC/src/Maslow"), "-o", str(binary)], check=True)
            subprocess.run([str(binary)], check=True)


    def test_production_acquisition_rolls_retries_and_bounds_work(self):
        root = Path(__file__).resolve().parents[3]
        source = (root / "FluidNC/src/Maslow/Calibration.cpp").read_text(encoding="utf-8-sig")
        production = extract_function(source, "bool Calibration::take_measurement_avg_with_check(")
        program = r"""
#include "CalibrationSampleWindow.h"
#include "MaslowEnums.h"
#include <array>
#include <vector>
#include <string>
#include <sstream>
#include <cassert>
#include <cmath>
#include <algorithm>
using std::abs;
using std::max;
std::vector<std::string> logs;
#define log_info(x) do { std::ostringstream s; s << x; logs.push_back(s.str()); } while(false)
#define log_error(x) log_info(x)
constexpr float BOOTSTRAP17_REPEATABILITY_MM = 15;
enum class State { Homing, Alarm };
struct { State current=State::Homing; void set_state(State value){current=value;} } sys;
struct Machine { void eStop(const char*) {} } Maslow;
struct Kinematics {
    float computeTL(float,float,int) {return 100;}
    float computeTR(float,float,int) {return 100;}
    float computeBL(float,float,int) {return 100;}
    float computeBR(float,float,int) {return 100;}
    float getTlZ() {return 0;} float getTrZ() {return 0;}
    float getBlZ() {return 0;} float getBrZ() {return 0;}
} kinematics;
Kinematics* getKinematics() {return &kinematics;}
class Calibration {
public:
    struct {int run=0, retries=0; float samples[4][4]={};} measurementState;
    bool calibrationBootstrap17=true, horizontalMeasurementFault=false;
    bool horizontalSamplingReferenceInvalid=false;
    int retractCurrentThreshold=850, currentState=CALIBRATION_IN_PROGRESS;
    float calibration_data[17][4]={}, calibrationGrid[17][2]={};
    float coarseStepX=150, coarseStepY=50;
    int resets=0;
    bool take_measurement_avg_with_check(int,int);
    std::vector<std::array<float,4>> input;
    std::vector<int> runs;
    unsigned next=0;
    bool take_measurement(float result[4],int,int run,int,int) {
        if(next==input.size()) return false; // Nonblocking acquisition pending.
        runs.push_back(run);
        for(int arm=0;arm<4;++arm) result[arm]=input[next][arm];
        ++next;
        return true;
    }
    void resetCalibrationState() {measurementState={}; horizontalSamplingReferenceInvalid=false; ++resets;}
    void requestStateChange(int state) {currentState=state;}
    bool adjustFrameSizeToMatchFirstMeasurement() {return true;}
    bool computeXYfromLengths(float,float,float& x,float& y) {x=0;y=0;return true;}
    float measurementToXYPlane(float value,float) {return value;}
};
PRODUCTION
void feed(Calibration& c,float value) {c.input.push_back({value,200,300,400});}
int main() {
    // One stale retained snapshot needs only one replacement, not six new pulls.
    Calibration c;
    for(float v : {90.f,95.f,80.f,100.f,101.f,102.f,103.f}) feed(c,v);
    for(int i=0;i<6;++i) assert(!c.take_measurement_avg_with_check(1,UP));
    assert(c.measurementState.run==5 && c.measurementState.retries==1);
    assert(c.take_measurement_avg_with_check(1,UP));
    assert(c.next==7 && c.runs==std::vector<int>({0,1,2,3,4,5,5}));
    assert(c.calibration_data[1][0]==101.5f && c.calibration_data[1][3]==400);
    assert(c.measurementState.run==0 && c.measurementState.retries==0);
    bool correctLabel=false;
    for(const auto& line:logs) if(line.find("axis=TL")!=std::string::npos) correctLabel=true;
    assert(correctLabel);
    // A pending asynchronous acquisition must not shift or consume the window.
    auto saved=c.measurementState.samples[0][0];
    assert(!c.take_measurement_avg_with_check(2,UP));
    assert(c.measurementState.run==0 && c.measurementState.samples[0][0]==saved);
    // Continuous slow drift was accepted by the old adjacent-only gate.
    Calibration drift;
    for(int i=0;i<61;++i) feed(drift,100.f+10.f*i);
    for(int i=0;i<60;++i) {
        assert(!drift.take_measurement_avg_with_check(1,UP));
        if(i<59) assert(drift.resets==0);
    }
    assert(drift.next==60 && drift.resets==1 && drift.currentState==EXTENDEDOUT);
    assert(drift.calibration_data[1][0]==0); // Never saved unstable data.
    // An invalid new snapshot aborts rather than poisoning or averaging the window.
    Calibration invalid;
    feed(invalid,std::numeric_limits<float>::quiet_NaN());
    assert(!invalid.take_measurement_avg_with_check(1,UP));
    assert(invalid.resets==1 && invalid.currentState==EXTENDEDOUT);
    // Existing tightening-fault propagation still resets and exits.
    Calibration fault;
    fault.horizontalMeasurementFault=true;
    assert(!fault.take_measurement_avg_with_check(1,UP));
    assert(fault.resets==1 && !fault.horizontalMeasurementFault);
    // New fixed-reference failures must not enter the position-saving extended state.
    Calibration invalidReference;
    invalidReference.horizontalMeasurementFault=true;
    invalidReference.horizontalSamplingReferenceInvalid=true;
    assert(!invalidReference.take_measurement_avg_with_check(1,UP));
    assert(invalidReference.resets==1 && invalidReference.currentState==UNKNOWN);
    assert(sys.current==State::Alarm);
    assert(invalidReference.calibration_data[1][0]==0);
    // Precision mode keeps its stricter 2.5 mm limit.
    Calibration precision;
    precision.calibrationBootstrap17=false;
    for(float v : {100.f,100.f,100.f,101.f,102.f,103.f}) feed(precision,v);
    for(int i=0;i<6;++i) assert(!precision.take_measurement_avg_with_check(1,UP));
    assert(precision.measurementState.retries==1);
}
""".replace("PRODUCTION", production)
        with tempfile.TemporaryDirectory(prefix="maslow-acquisition-") as directory:
            cpp=Path(directory)/"acquisition.cpp"
            binary=Path(directory)/("acquisition.exe" if os.name=="nt" else "acquisition")
            cpp.write_text(program,encoding="utf-8")
            subprocess.run(compiler_command(root)+["-std=c++11",str(cpp),
                "-I",str(root/"FluidNC/src/Maslow"),"-o",str(binary)],check=True)
            subprocess.run([str(binary)],check=True)


if __name__ == "__main__":
    unittest.main()
