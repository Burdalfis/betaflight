"""Exercise timestamp rollover and qualification boundaries in the actual C++ helpers."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


class TimingHelpers(unittest.TestCase):
    def compile_and_run(self, source):
        compiler = shutil.which("g++")
        self.assertIsNotNone(compiler, "g++ is required to test the firmware helpers")
        with tempfile.TemporaryDirectory(prefix="adxl-timing-") as folder:
            path = Path(folder)
            (path / "test.cpp").write_text(source)
            subprocess.run([compiler, "-std=c++11", "-Wall", "-Wextra", "-Werror",
                            "-fsanitize=undefined", "-I" + str(Path(__file__).parent / "include"),
                            str(path / "test.cpp"), "-o", str(path / "test")],
                           check=True, capture_output=True, text=True)
            result = subprocess.run([str(path / "test")], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_actual_cpp_timing_and_pwm_guards(self):
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
    assert(!qualifiedTime(true, 0, false, false, 0, false)); // A foreign edge with the DRDY line LOW.
    assert(qualifiedTime(true, 25, false, false, 600, true));
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
        self.compile_and_run(source)

    def test_periodic_readback_yields_without_skipping_registers(self):
        self.compile_and_run(r'''
#include <cassert>
#include "logger_protocol.h"
using namespace adxl_logger;
int main() {
    constexpr uint64_t interval = 1000000;
    const uint64_t base = UINT64_C(0xfffffff0);
    PeriodicRegisterScan scan;
    scan.restart(base, interval);
    scan.accept(base, 10, interval); // An idle scan cannot skip a register.
    assert(scan.index() == 0);
    assert(!scan.ready(base + interval - 1, false));
    assert(!scan.ready(base + interval, true)); // A pending sample wins.
    uint64_t now = base + interval + 10; // Deadline crosses a 32-bit clock wrap.
    for (uint8_t index = 0; index < 10; ++index) {
        assert(scan.ready(now, false));
        assert(scan.index() == index);
        assert(!scan.ready(now + 1, true));
        assert(scan.index() == index); // Interleaving preserves the next check.
        now += 17;
        scan.accept(now, 10, interval);
    }
    assert(scan.index() == 0);
    assert(!scan.ready(now + interval - 1, false));
    assert(scan.ready(now + interval, false));
    scan.accept(now + interval + 25, 10, interval);
    assert(scan.index() == 1);
    scan.restart(now + interval + 30, interval); // Fault/retry starts a fresh audit.
    assert(scan.index() == 0);
    assert(!scan.ready(now + 2 * interval + 29, false));
    assert(scan.ready(now + 2 * interval + 30, false));
}
''')

    def test_fifo_empty_high_water_overrun_and_stalled_pop(self):
        self.compile_and_run(r'''
#include <cassert>
#include "logger_protocol.h"
using namespace adxl_logger;
int main() {
    assert(fifoDecision(0, 0x02) == FifoDecision::Empty);
    assert(fifoDecision(0, 0x03) == FifoDecision::Discard);
    for (unsigned depth = 1; depth <= 30; ++depth) {
        assert(fifoDecision(depth, 0x82) == FifoDecision::Read);
        assert(fifoDecision(depth, 0x83) == FifoDecision::Discard);
        assert(fifoPopAccepted(depth, 0x82, depth - 1, 0x02, 625));
        assert(!fifoPopAccepted(depth, 0x82, depth - 1, 0x02, 626));
    }
    assert(!fifoPopAccepted(0, 0x82, 0, 0x02, 100)); // Never read empty FIFO.
    assert(!fifoPopAccepted(31, 0x82, 30, 0x02, 100));
    assert(!fifoPopAccepted(1, 0x82, 31, 0x02, 100));
    assert(!fifoPopAccepted(1, 0x82, 1, 0x03, 100)); // Post-pop overrun.
    assert(!fifoPopAccepted(1, 0x83, 0, 0x02, 100)); // Pop can clear old overrun.
    assert(fifoDecision(31, 0x82) == FifoDecision::Discard);
    assert(!fifoOverrunEvidence(31, 0x82)); // High water is not proven loss.
    assert(fifoOverrunEvidence(32, 0x82));
    assert(fifoOverrunEvidence(1, 0x83));
    assert(!fifoPopAccepted(1, 0x82, 0, 0x02, UINT64_C(0x100000000)));
}
''')


if __name__ == "__main__":
    unittest.main()
