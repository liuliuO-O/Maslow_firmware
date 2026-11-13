"""Exercise the production pull_tight() stall guard with simulated motor I/O.

Covers the failure mode seen on the bottom-right belt: the tension decision could
not distinguish "taut" from "jammed", so a jam kept receiving force until the
driver protection tripped.  These cases pin the new behaviour:

* a jam at full duty latches TIGHT_STALLED and cuts the output,
* a driver with no current latches TIGHT_NO_DRIVE and cuts the output,
* a belt that is still travelling at full duty is not flagged,
* a taut belt holds its duty instead of ramping up again on every call.
"""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

from test_calibration_bootstrap import compiler_command, extract_function


HARNESS = r'''
#include <cassert>
#include <cmath>
#include <cstdint>
#include <string>

static unsigned long clockMs = 1000;
unsigned long millis() { return clockMs; }

static int randChoice = 1;  // always take the ramp step
int random(int, int) { return randChoice; }
template <class T> T pickMin(T a, T b) { return a < b ? a : b; }
#define min(a, b) pickMin(a, b)

struct String {
    std::string s;
    String(const char* v = "") : s(v) {}
    const char* c_str() const { return s.c_str(); }
};
struct Labels {
    String axis_id_to_label(int) { return String("BR"); }
} Maslow;

static int warnings = 0;
#define log_warn(x) do { ++warnings; } while (false)
#define log_debug(x) do { } while (false)

struct Motor {
    int pwm = 0;
    int current = 0;
    void backward(uint16_t speed) { pwm = -static_cast<int>(speed); }
    void stop() { pwm = 0; }
    int  readCurrent() { return current; }
};

struct MotorUnit {
    enum TightOutcome : uint8_t { TIGHT_PULLING, TIGHT_OK, TIGHT_STALLED, TIGHT_NO_DRIVE };

    Motor    motor;
    double   pos          = 1000.0;
    int      _commandPWM  = 0;
    int      _encoderAddress = 0;
    uint16_t retract_speed = 0;
    float    retract_baseline = 700;
    int      incrementalThreshold = 125;
    int      incrementalThresholdHits = 0;
    float    alpha = 0.2f;
    unsigned long lastCallToRetract = 0;
    TightOutcome  tightOutcome_ = TIGHT_PULLING;
    uint16_t      tightHoldPwm  = 0;
    unsigned long stallCheckMs  = 0;
    double        stallCheckPos = 0;
    uint32_t      stallMs       = 0;
    static constexpr uint16_t stallMinPwm      = 1000;
    static constexpr double   stallMinMotionMm = 0.4;
    static constexpr uint32_t stallWindowMs    = 600;
    static constexpr uint16_t tightHoldPwmCap  = 700;

    bool   tightFault() const { return tightOutcome_ == TIGHT_STALLED || tightOutcome_ == TIGHT_NO_DRIVE; }
    double getPosition() { return pos; }
    double getMotorCurrent() { return motor.current; }
    void   stop() { motor.stop(); _commandPWM = 0; }
    bool   pull_tight(int currentThreshold);
};

BLOCK

// Advance 4 ms and call pull_tight once (the production call cadence).
static bool tick(MotorUnit& unit, int threshold = 1300) {
    clockMs += 4;
    return unit.pull_tight(threshold);
}

int main() {
    // 1) Jam: frozen belt, load well below the tension threshold.
    {
        MotorUnit jam;
        jam.motor.current = 800;
        int guard = 0;
        while (!jam.tightFault() && guard++ < 10000) tick(jam);
        assert(jam.tightFault());
        assert(jam.tightOutcome_ == MotorUnit::TIGHT_STALLED);
        assert(jam.motor.pwm == 0);          // output cut at the fault
        assert(warnings == 1);
        // Latched: no further force, ever, until reset.
        for (int i = 0; i < 500; ++i) { tick(jam); assert(jam.motor.pwm == 0); }
        assert(jam.retract_speed == 0);
    }

    // 2) Driver with no output: full duty, zero current, frozen belt.
    {
        MotorUnit dead;
        dead.motor.current = 0;
        int guard = 0;
        while (!dead.tightFault() && guard++ < 10000) tick(dead);
        assert(dead.tightOutcome_ == MotorUnit::TIGHT_NO_DRIVE);
        assert(dead.motor.pwm == 0);
    }

    // 3) A belt still travelling at full duty is not a jam.
    {
        MotorUnit moving;
        moving.motor.current = 800;
        int guard = 0;
        while (guard++ < 3000) {
            moving.pos += 0.5;  // 0.5 mm per 4 ms call = 125 mm/s
            tick(moving);
            assert(!moving.tightFault());
        }
    }

    // 4) A taut belt reports success once and then holds that duty.
    {
        MotorUnit taut;
        taut.motor.current = 500;
        int guard = 0;
        bool tight = false;
        while (guard++ < 10000 && !tight) {
            taut.pos += 0.2;
            if (guard > 400) taut.motor.current = 1350;  // crosses the 1300 threshold
            tight = tick(taut);
        }
        assert(tight);
        assert(taut.tightOutcome_ == MotorUnit::TIGHT_OK);
        assert(taut.motor.pwm == 0);  // stop() ran on the decision
        clockMs += 4;
        taut.pull_tight(1300);
        const int held = -static_cast<int>(taut.tightHoldPwm);
        assert(held < 0 && held >= -700);
        assert(taut.motor.pwm == held);
        // Repeat calls keep the same duty: no re-ramp, no extra inward nudge.
        for (int i = 0; i < 200; ++i) {
            tick(taut);
            assert(taut.motor.pwm == held);
            assert(!taut.tightFault());
        }
    }

    // 5) Reset clears the latch so a later phase can tension again.
    {
        MotorUnit unit;
        unit.motor.current = 800;
        int guard = 0;
        while (!unit.tightFault() && guard++ < 10000) tick(unit);
        unit.tightOutcome_ = MotorUnit::TIGHT_PULLING;
        unit.stallMs = 0;
        unit.stallCheckMs = 0;
        unit.retract_speed = 0;
        tick(unit);
        assert(!unit.tightFault());
        assert(unit.motor.pwm < 0);
    }
    return 0;
}
'''


class TensionStallGuardTest(unittest.TestCase):
    def test_pull_tight_latches_jam_and_holds_tension(self):
        root = Path(__file__).resolve().parents[3]
        source = (root / 'FluidNC/src/Maslow/MotorUnit.cpp').read_text(encoding='utf-8-sig')
        block = extract_function(source, 'bool MotorUnit::pull_tight(')
        program = HARNESS.replace('BLOCK', block)
        with tempfile.TemporaryDirectory(prefix='maslow-tension-guard-') as directory:
            cpp = Path(directory) / 'test.cpp'
            binary = Path(directory) / ('test.exe' if os.name == 'nt' else 'test')
            cpp.write_text(program, encoding='utf-8')
            subprocess.run(compiler_command(root) + ['-std=c++11', str(cpp), '-o', str(binary)], check=True)
            subprocess.run([str(binary)], check=True)


if __name__ == '__main__':
    unittest.main()
