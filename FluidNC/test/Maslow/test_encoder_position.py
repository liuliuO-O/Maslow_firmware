import subprocess
import tempfile
import textwrap
import unittest
from pathlib import Path


class EncoderPositionTest(unittest.TestCase):
    def test_unwrap_uses_motor_direction_only_for_ambiguous_motion(self):
        root = Path(__file__).resolve().parents[2]
        source = textwrap.dedent(r'''
            #include "EncoderPosition.h"
            #include <cassert>

            int main() {
                using MaslowEncoder::unwrapDelta;

                // Normal wrap-around while stationary/manual movement.
                assert(unwrapDelta(4000, 100, 0) == 196);
                assert(unwrapDelta(100, 4000, 0) == -196);

                // Samples from the observed one-direction retract sequence.
                // Increasing encoder counts resolves both sides consistently.
                assert(unwrapDelta(2964, 1238, 1) == 2370);
                assert(unwrapDelta(1238, 2996, 1) == 1758);

                // Extend uses decreasing encoder counts.
                assert(unwrapDelta(1238, 2996, -1) == -2338);
                assert(unwrapDelta(2996, 1416, -1) == -1580);

                // Small backlash must not be expanded into a full turn.
                assert(unwrapDelta(1000, 990, 1) == -10);
                assert(unwrapDelta(1000, 1010, -1) == 10);
            }
        ''')
        with tempfile.TemporaryDirectory() as tmp:
            src = Path(tmp) / 'test.cpp'
            exe = Path(tmp) / 'test'
            src.write_text(source)
            subprocess.run([
                'c++', '-std=c++17', '-I', str(root / 'src' / 'Maslow'), str(src), '-o', str(exe)
            ], check=True)
            subprocess.run([str(exe)], check=True)


if __name__ == '__main__':
    unittest.main()
