import os
from test_calibration_bootstrap import compiler_command
"""Exercise the production horizontal initial-tightening block with staged completions."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


class HorizontalTighteningTest(unittest.TestCase):
    def test_completed_axes_keep_tension_and_bottom_outputs_are_cleared(self):
        source = (Path(__file__).resolve().parents[2] / 'src/Maslow/Calibration.cpp').read_text(encoding='utf-8-sig')
        start = source.index('            if (!initial_tension_complete) {')
        end = source.index('\n            diagnosticTightMask =', start)
        block = source[start:end]
        harness = r'''
#include <cassert>
#include <algorithm>
#include <array>
enum { _TL, _TR, _BL, _BR };
int tick;
struct MotorUnit {
    enum TightOutcome : unsigned char { TIGHT_PULLING, TIGHT_OK, TIGHT_STALLED, TIGHT_NO_DRIVE };
};
struct Axis {
    int finish = 0, calls = 0, stops = 0, pwm = 0;
    double target = -1, position = 42;
    bool pull_tight(int) {
        ++calls;
        pwm = -500;
        if (tick == finish) { stop(); return true; }
        return false;
    }
    bool tightFault() { return false; }
    MotorUnit::TightOutcome tightOutcome() { return MotorUnit::TIGHT_PULLING; }
    void stop() { ++stops; pwm = 0; }
    double getPosition() { return position; }
    void setTarget(double p) { target = p; }
};
struct { Axis axis[4]; } Maslow;
bool failHorizontalSampling(const char*, unsigned, bool = false) { return false; }
void diagnosticStage(const char*) {}
bool tight[4];
bool initial_tension_complete;
unsigned diagnosticTightMask;
bool step(int current) {
BLOCK
    return true;
}
int main() {
    // All 24 possible completion orders, including each bottom axis finishing first.
    std::array<int,4> order = {0,1,2,3};
    do {
        initial_tension_complete = false;
        for (int i=0;i<4;++i) { tight[i]=false; Maslow.axis[i]=Axis{}; }
        for (int i=0;i<4;++i) Maslow.axis[order[i]].finish=i;
        for (tick=0;tick<4;++tick) {
            assert(!step(1000));
            for (int i=0;i<4;++i) {
                auto& axis=Maslow.axis[i];
                assert(axis.calls == tick+1);
                if (axis.finish < tick && tick < 3) assert(axis.pwm == -500);
            }
            assert(initial_tension_complete == (tick==3));
        }
        assert(tight[_TL] && tight[_TR] && !tight[_BL] && !tight[_BR]);
        assert(Maslow.axis[_BL].stops==2 && Maslow.axis[_BR].stops==2);
        assert(Maslow.axis[_TL].target==42 && Maslow.axis[_TR].target==42);
        assert(Maslow.axis[_BL].target==-1 && Maslow.axis[_BR].target==-1);
        assert(step(1000)); // Initial phase no longer issues commands.
        for(int i=0;i<4;++i) assert(Maslow.axis[i].calls==4);
    } while(std::next_permutation(order.begin(),order.end()));
}
'''.replace('BLOCK', block)
        with tempfile.TemporaryDirectory() as directory:
            cpp = Path(directory) / 'test.cpp'
            binary = Path(directory) / ('test.exe' if os.name == 'nt' else 'test')
            cpp.write_text(harness, encoding='utf-8')
            subprocess.run(compiler_command(Path(__file__).resolve().parents[3]) + ['-std=c++11', str(cpp), '-o', str(binary)], check=True)
            subprocess.run([str(binary)], check=True)


if __name__ == '__main__':
    unittest.main()
