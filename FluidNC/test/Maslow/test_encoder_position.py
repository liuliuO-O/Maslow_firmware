import subprocess
import tempfile
import textwrap
import unittest
from pathlib import Path


class EncoderPositionTest(unittest.TestCase):
    def test_unwrap_uses_shortest_delta_across_zero(self):
        root = Path(__file__).resolve().parents[2]
        source = textwrap.dedent(r'''
            #include "EncoderPosition.h"
            #include <cassert>

            int main() {
                using MaslowEncoder::unwrapDelta;

                // Normal wrap-around while stationary/manual movement.
                assert(unwrapDelta(4000, 100) == 196);
                assert(unwrapDelta(100, 4000) == -196);
                assert(unwrapDelta(1000, 990) == -10);
                assert(unwrapDelta(1000, 1010) == 10);

                // Never invent a nearly complete revolution from one sample.
                assert(unwrapDelta(500, 4032) == -564);
                assert(unwrapDelta(4032, 500) == 564);
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
