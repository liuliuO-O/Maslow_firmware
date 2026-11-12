"""Run production calibration grid/state transitions with simulated I/O only.

This checks software sequencing, failure handling and reruns. It does not claim
that motors have moved or that physical calibration accuracy has been verified.
Set MASLOW_SOURCE_ROOT for this temporary draft, or place it in FluidNC/test/Maslow.
CXX accepts a compiler executable or a command such as '".../zig.exe" c++'.
"""

from pathlib import Path
import json
import os
import re
import shlex
import shutil
import subprocess
import tempfile
import unittest


def extract_function(source, signature):
    """Copy a complete production function, ignoring braces in strings/comments."""
    start = source.index(signature)
    brace = source.index("{", start)
    depth = 0
    token = re.compile(r'//[^\n]*|/\*[\s\S]*?\*/|"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'|[{}]')
    for match in token.finditer(source, brace):
        if match.group() == "{":
            depth += 1
        elif match.group() == "}":
            depth -= 1
            if depth == 0:
                return source[start:match.end()]
    raise ValueError("Unterminated production function: " + signature)


def compiler_command(root):
    configured = os.environ.get("CXX")
    if configured:
        if configured.lstrip().startswith("["):
            command = json.loads(configured)
        elif Path(configured).is_file():
            command = [configured]
        else:
            command = shlex.split(configured, posix=os.name != "nt")
            command = [part[1:-1] if len(part) >= 2 and part[0] == part[-1] == '"' else part
                       for part in command]
    else:
        compiler = shutil.which("c++") or shutil.which("g++") or shutil.which("clang++")
        zig = root / ".pio/host-tools/ziglang/zig.exe"
        if not compiler and zig.is_file():
            compiler = str(zig)
        if not compiler:
            raise RuntimeError("A host C++ compiler is required; set CXX or install the project host tool")
        command = [compiler]
    if Path(command[0]).name.lower() in ("zig", "zig.exe") and len(command) == 1:
        command.append("c++")
    return command


HARNESS = r'''
#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include "CalibrationStabilityWindow.h"
#include "SavedAnchorRecovery.h"
using std::max;
using std::min;
unsigned long testClock = 100;
unsigned long millis() { return testClock; }
constexpr int CALIBRATION_GRID_SIZE_MAX = 256;
ENUMS
CONSTANTS
std::string M = "Maslow";
std::vector<std::string> logLines;
#define log_info(expr) do { std::ostringstream s; s << expr; logLines.push_back(s.str()); } while(false)
#define log_error(expr) log_info(expr)
#define log_warn(expr) log_info(expr)
#define log_debug(expr) log_info(expr)
void serviceCalibrationWatchdogs(bool = false) {}
namespace WebUI { enum class AuthenticationLevel { LEVEL_ADMIN }; }
enum class Error { Ok = 0, FileWriteError = 1 };
int allChannels = 0;
enum class State { Idle, Homing, Alarm };
struct System {
    State current = State::Homing;
    void set_state(State value) { current = value; }
    State state() const { return current; }
} sys;
struct Axis {
    float position = 1000.0f;
    float target = 1000.0f;
    float getPosition() const { return position; }
    float getTarget() const { return target; }
    void setTarget(float value) { target = value; }
    void stop() {}
    void reset() {}
    unsigned getEncoderReadFailures() const { return 0; }
};
struct Machine {
    float workAreaX = 500.0f, workAreaY = 300.0f;
    float x = 0.0f, y = 0.0f;
    Axis axis[ARM_COUNT];
    void stopMotors() {}
    void stop() {}
    void resetUpdateWatchdog() {}
} Maslow;
struct FakeKinematics {
    float getTlX() const { return 0.0f; }
    float getTlY() const { return 4000.0f; }
    float getTrX() const { return 6000.0f; }
    float getTrY() const { return 4000.0f; }
    float getBlY() const { return 0.0f; }
    float getBrX() const { return 6000.0f; }
} testKinematics;
FakeKinematics* getKinematics() { return &testKinematics; }
std::vector<int> measured;
std::vector<int> recomputed;
std::vector<int> moved;
int failingFitCount = -1;
bool failingSave = false;
int saveCalls = 0;
Error execute_line(char* command, int, WebUI::AuthenticationLevel) {
    assert(std::string(command) == "$CO");
    ++saveCalls;
    return failingSave ? Error::FileWriteError : Error::Ok;
}
HEADER
MEASUREMENT_STORAGE
PRODUCTION

Calibration::Calibration() { orientation = HORIZONTAL; calibrationInProgress = false; }
void Calibration::diagnosticUpdate() {}
void Calibration::diagnosticStage(const char*) {}
bool Calibration::detectOrientation() { return true; }
void Calibration::hold(unsigned long) {}
void Calibration::clearMotorOverrides() {}
void Calibration::logClbmMeasurements(int) const {}
bool Calibration::updateExtendDistanceFromAnchors() { return true; }
bool Calibration::requestStateChange(int state) {
    currentState = state;
    calibrationInProgress = (state == CALIBRATION_IN_PROGRESS);
    if (state == READY_TO_CUT || state == EXTENDEDOUT) sys.set_state(State::Idle);
    return true;
}
bool Calibration::take_measurement_avg_with_check(int point, int) {
    assert(point >= 0 && point <= pointCount);
    assert(point == static_cast<int>(measured.size()));
    measured.push_back(point);
    // The physical sampler is replaced, but every software waypoint must still
    // be visited. These values are never used to claim real fit accuracy.
    for (int arm = 0; arm < ARM_COUNT; ++arm) calibration_data[point][arm] = 1000 + point + arm;
    return true;
}
bool Calibration::move_with_slack(double fromX, double fromY, double toX, double toY) {
    assert(waypoint > 0 && waypoint <= pointCount);
    assert(std::isfinite(fromX) && std::isfinite(fromY));
    assert(std::isfinite(toX) && std::isfinite(toY));
    moved.push_back(waypoint);
    return true;
}
bool Calibration::recomputeAnchorsWithLevenbergMarquardt(int count) {
    recomputed.push_back(count);
    lastRecomputePassed = count != failingFitCount;
    return lastRecomputePassed;
}

void clearTrace() {
    measured.clear(); recomputed.clear(); moved.clear(); logLines.clear();
    saveCalls = 0; failingFitCount = -1; failingSave = false;
}
bool completed() {
    return std::find(logLines.begin(), logLines.end(), "校准完成 | 锚点已保存 | Find Anchors complete") != logLines.end();
}
void prepare(Calibration& c) {
    c.currentState = CALIBRATION_IN_PROGRESS;
    c.calibrationInProgress = true;
    c.measurementInProgress = true;
    c.waypoint = 0;
    c.recomputeCountIndex = 0;
    c.lastRecomputePassed = false;
    assert(c.generate_calibration_grid());
    assert(c.pointCount == 16);
    assert(c.recomputePoints[0] == 5);
    assert(c.recomputePoints[1] == 14);
    assert(c.recomputePoints[2] == 16);
    // The initial sampler normally locates these relative to the actual start.
    // This fixture seeds them because sampler hardware is outside this test.
    const float sx = c.coarseStepX, sy = c.coarseStepY;
    const float coarse[6][2] = {{0,0},{sx,0},{sx,sy},{0,sy},{-sx,sy},{-sx,0}};
    for (int i = 0; i < 6; ++i) {
        c.calibrationGrid[i][0] = coarse[i][0];
        c.calibrationGrid[i][1] = coarse[i][1];
    }
}
void drain(Calibration& c) {
    for (int tick = 0; tick < 200 && c.currentState == CALIBRATION_IN_PROGRESS; ++tick) {
        testClock += 200;
        c.calibration_loop();
    }
    assert(c.currentState != CALIBRATION_IN_PROGRESS);
}
void verifySuccess(Calibration& c) {
    assert(measured.size() == 17);
    for (int i = 0; i < 17; ++i) assert(measured[i] == i);
    assert((recomputed == std::vector<int>{6,15,17}));
    assert(moved.size() == 16);
    for (int i = 0; i < 16; ++i) assert(moved[i] == i + 1);
    assert(saveCalls == 1);
    assert(c.currentState == READY_TO_CUT);
    assert(sys.state() == State::Idle);
    assert(completed());
}
int main(int argc, char** argv) {
    assert(argc == 2);
    std::string scenario = argv[1];
    Calibration c;
    clearTrace();
    if (scenario == "grid") {
        // Saved grid_size=9 and automatic dimensions must not defeat Bootstrap17.
        Maslow.workAreaX = 5000; Maslow.workAreaY = 4000;
        c.calibrationBootstrap17 = true;
        c.calibrationGridSize = 9;
        prepare(c);
        assert(std::any_of(logLines.begin(), logLines.end(), [](const std::string& line) {
            return line.find("profile=bootstrap17 points=17 grid=3") != std::string::npos;
        }));
        assert(c.calibrationGrid[16][0] == 0 && c.calibrationGrid[16][1] == 0);
        c.resetCalibrationState();
        c.calibrationBootstrap17 = false;
        c.calibrationGridSize = 5;
        c.calibration_grid_width_mm_X = 1200;
        c.calibration_grid_height_mm_Y = 600;
        assert(c.generate_calibration_grid());
        assert(c.calibrationGridSize == 5);
        assert(c.pointCount == 32); // Standard 5x5 path has 33 measurement rows.
        c.resetCalibrationState();
    } else if (scenario == "success") {
        c.calibrationBootstrap17 = true;
        prepare(c); drain(c); verifySuccess(c);
        // Repeat the whole software flow with the same object after production reset.
        clearTrace(); prepare(c); drain(c); verifySuccess(c);
    } else if (scenario == "reset") {
        c.resetCalibrationState(); // Must be safe before any memory allocation.
        c.resetCalibrationState();
        prepare(c);
        c.calibration_loop();
        c.resetCalibrationState();
        c.resetCalibrationState();
        clearTrace(); prepare(c); drain(c); verifySuccess(c);
    } else if (scenario == "save_failure") {
        prepare(c); failingSave = true; drain(c);
        assert(measured.size() == 17);
        assert((recomputed == std::vector<int>{6,15,17}));
        assert(saveCalls == 1);
        assert(c.currentState != READY_TO_CUT);
        assert(!completed());
    } else if (scenario.rfind("fit_failure_", 0) == 0) {
        const int failedAt = std::stoi(scenario.substr(12));
        prepare(c); failingFitCount = failedAt; drain(c);
        assert(static_cast<int>(measured.size()) == failedAt);
        assert(recomputed.back() == failedAt);
        assert(saveCalls == 0);
        assert(c.currentState != READY_TO_CUT);
        assert(!completed());
        clearTrace(); prepare(c); drain(c); verifySuccess(c);
    } else {
        throw std::runtime_error("Unknown scenario");
    }
    c.deallocateCalibrationMemory();
    std::cout << "PASS " << scenario << '\n';
}
'''


class CalibrationBootstrapTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        configured = os.environ.get("MASLOW_SOURCE_ROOT")
        root = Path(configured) if configured else Path(__file__).resolve().parents[3]
        source = (root / "FluidNC/src/Maslow/Calibration.cpp").read_text(encoding="utf-8-sig")
        header = (root / "FluidNC/src/Maslow/Calibration.h").read_text(encoding="utf-8-sig")
        enums = (root / "FluidNC/src/Maslow/MaslowEnums.h").read_text(encoding="utf-8-sig")
        declaration = header[header.index("class Calibration {"):].replace("private:", "public:")
        signatures = [
            "bool Calibration::generate_calibration_grid()",
            "void Calibration::calibration_loop()",
            "int Calibration::get_direction(",
            "void Calibration::allocateCalibrationMemory()",
            "void Calibration::deallocateCalibrationMemory()",
            "void Calibration::resetMeasurementState()",
            "void Calibration::resetHorizontalSampling()",
            "void Calibration::resetCalibrationState()",
        ]
        production = "\n\n".join(extract_function(source, item) for item in signatures)
        storage_start = source.find("static float** measurements")
        storage_end = source.index("bool Calibration::take_measurement_avg_with_check(")
        storage = source[storage_start:storage_end] if storage_start >= 0 else ""
        constants = "\n".join(re.findall(r"^\s*(?:constexpr double FITNESS_|static constexpr float BOOTSTRAP17_).*?;", source, re.MULTILINE))
        program = (HARNESS.replace("ENUMS", enums)
                   .replace("CONSTANTS", constants)
                   .replace("HEADER", declaration)
                   .replace("MEASUREMENT_STORAGE", storage)
                   .replace("PRODUCTION", production))
        cls._directory = tempfile.TemporaryDirectory(prefix="maslow-calibration-test-")
        cls.addClassCleanup(cls._directory.cleanup)
        cpp = Path(cls._directory.name) / "calibration_bootstrap.cpp"
        cls.binary = Path(cls._directory.name) / ("calibration_bootstrap.exe" if os.name == "nt" else "calibration_bootstrap")
        cpp.write_text(program, encoding="utf-8")
        command = compiler_command(root) + ["-std=c++17", "-O0", "-g", "-I", str(root / "FluidNC/src/Maslow"), str(cpp), "-o", str(cls.binary)]
        result = subprocess.run(command, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        if result.returncode:
            raise AssertionError("Production calibration harness did not compile:\n" + result.stdout)

    def run_scenario(self, scenario):
        result = subprocess.run([str(self.binary), scenario], text=True,
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        self.assertEqual(result.returncode, 0, result.stdout or ("Failed scenario " + scenario))
        self.assertIn("PASS " + scenario, result.stdout)

    def test_bootstrap_forces_17_rows_and_standard_mode_still_works(self):
        self.run_scenario("grid")

    def test_visits_17_points_fits_6_15_17_saves_and_can_repeat(self):
        self.run_scenario("success")

    def test_reset_without_allocation_and_after_interruption(self):
        self.run_scenario("reset")

    def test_save_failure_cannot_report_success(self):
        self.run_scenario("save_failure")

    def test_each_fit_failure_stops_without_save_and_allows_rerun(self):
        for count in (6, 15, 17):
            with self.subTest(measurements=count):
                self.run_scenario("fit_failure_" + str(count))


if __name__ == "__main__":
    unittest.main()
