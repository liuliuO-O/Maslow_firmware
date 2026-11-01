"""Host-side logging-only tests using the diagnostic methods extracted from production C++."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


class CalibrationDiagnosticsTest(unittest.TestCase):
    def test_logging_does_not_control_motion(self):
        compiler = shutil.which('c++')
        self.assertIsNotNone(compiler, 'A host C++ compiler is required')
        root = Path(__file__).resolve().parents[2]
        source = (root / 'src/Maslow/Calibration.cpp').read_text()
        methods = source[source.index('void Calibration::diagnosticStage('):
                         source.index('// -Maslow homing loop.')]
        header = (root / 'src/Maslow/Calibration.h').read_text()
        fields = header[header.index('    void diagnosticStage('):
                        header.index('    //Variables used for retracting state')]
        harness = r'''
#include <cassert>
#include <cstdint>
#include <cstring>
#include <string>
#define log_info(...) ((void)0)
#define log_error(...) ((void)0)
constexpr int ARM_COUNT = 4, CALIBRATION_IN_PROGRESS = 7;
unsigned long clockMs = 0;
unsigned long millis() { return clockMs; }
struct Axis {};
// Deliberately no stop/panic/error API: diagnostic methods must only read axes.
struct Machine { Axis axis[4]; } Maslow;
class Calibration {
public:
    bool calibrationInProgress = true;
    int currentState = CALIBRATION_IN_PROGRESS, waypoint = 0;
    bool overridesCleared = false;
    void clearMotorOverrides() { overridesCleared = true; }
FIELDS
};
METHODS
int main() {
    Calibration c;
    c.diagnosticUpdate();
    c.diagnosticStage("return");
    for (auto t : {10000UL, 20000UL, 30000UL, 90000UL, 180000UL}) {
        clockMs = t;
        c.diagnosticUpdate();
        c.diagnosticStage("return");
        assert(c.calibrationInProgress);
        assert(c.currentState == CALIBRATION_IN_PROGRESS);
        assert(c.waypoint == 0);
        assert(c.diagnosticPhaseStart == 0); // Same phase keeps elapsed time, never aborts.
    }
    c.waypoint = 1;
    c.diagnosticUpdate();
    c.diagnosticStage("return");
    assert(c.diagnosticPhaseStart == clockMs);
    auto lastLog = c.diagnosticLastLog;
    clockMs += 499;
    c.diagnosticUpdate();
    assert(c.diagnosticLastLog == lastLog);
    clockMs += 1;
    c.diagnosticUpdate();
    assert(c.diagnosticLastLog == clockMs);
    Calibration idle;
    idle.calibrationInProgress = false;
    idle.diagnosticUpdate();
    idle.diagnosticStage("unused");
    assert(idle.diagnosticPhase == nullptr);
    assert(idle.diagnosticLastLog == 0);
}

'''.replace('FIELDS', fields).replace('METHODS', methods)
        with tempfile.TemporaryDirectory() as directory:
            cpp = Path(directory) / 'test.cpp'
            binary = Path(directory) / 'test'
            cpp.write_text(harness)
            subprocess.run([compiler, '-std=c++11', str(cpp), '-o', str(binary)], check=True)
            subprocess.run([str(binary)], check=True)


if __name__ == '__main__':
    unittest.main()
