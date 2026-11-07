"""Exercise timestamp rollover and qualification boundaries in the actual C++ helpers."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


class TimingHelpers(unittest.TestCase):
    def test_actual_cpp_timing_and_pwm_guards(self):
        compiler = shutil.which("g++")
        self.assertIsNotNone(compiler, "g++ is required to test the firmware helpers")
        source = r'''
#include "logger_protocol.h"
#include <cassert>
using namespace adxl_logger;
int main() {
    MicrosClock clock;
    assert(clock.update(0xfffffff0U) == 0xfffffff0ULL);
    assert(clock.update(0x10U) == 0x100000010ULL);
    assert(clock.update(0xfffffff0U) == 0x1fffffff0ULL);
    assert(clock.update(0x10U) == 0x200000010ULL);
    assert(expandRecent(0x100000010ULL, 0xfffffff0U) == 0xfffffff0ULL);
    assert(expandRecent(0x200000010ULL, 0xfffffff0U) == 0x1fffffff0ULL);
    assert(expandRecent(100, 90) == 90);
    assert(expandRecent(100, 101) == 0); // No fabricated future timestamp at boot.
    assert(expandRecent(16, 0xfffffff0U) == 0); // No prior-wrap timestamp before first wrap.
    assert(qualifiedTime(true, 625, false, false));
    assert(!qualifiedTime(true, 626, false, false));
    assert(!qualifiedTime(false, 0, false, false));
    assert(!qualifiedTime(true, 0, true, false));
    assert(!qualifiedTime(true, 0, false, true));
    assert(!qualifiedTime(true, 25, false, false, 601)); // Transfer crossed the deadline.
    assert(qualifiedTime(true, 25, false, false, 600));
    assert(!qualifiedTime(true, 25, false, false, 10000000000ULL));
    assert(validPwmWidth(500) && validPwmWidth(2500));
    assert(!validPwmWidth(499) && !validPwmWidth(2501));
    assert(freshPwmAt(true, 1, 100000, 0, 1420));
    assert(!freshPwmAt(true, 1, 100001, 0, 1420));
    assert(!freshPwmAt(true, 1, 1000, 1001, 1420)); // Future rise.
    assert(!freshPwmAt(true, 1, 1000, 0, 1420)); // Fall after sample time.
    assert(freshPwmAt(true, 1, 1420, 0, 1420)); // Completed exactly at sample time.
    assert(!freshPwmAt(false, 1, 1420, 0, 1420));
    assert(!freshPwmAt(true, 0, 1420, 0, 1420));
}
'''
        with tempfile.TemporaryDirectory(prefix="adxl-timing-") as folder:
            path = Path(folder)
            (path / "test.cpp").write_text(source)
            subprocess.run([compiler, "-std=c++11", "-Wall", "-Wextra", "-Werror",
                            "-fsanitize=undefined", "-I" + str(Path(__file__).parent / "include"),
                            str(path / "test.cpp"), "-o", str(path / "test")],
                           check=True, capture_output=True, text=True)
            result = subprocess.run([str(path / "test")], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


if __name__ == "__main__":
    unittest.main()
