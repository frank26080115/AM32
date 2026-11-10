"""Exercise the actual sampler and DShot decoder with synthetic GPIO samples.

Run on Windows with Python and clang/lld installed:
    python tests/test_dshot_slow.py
Hardware registers are stubbed; packet parsing and CRC use production C code.
"""
import ctypes
import itertools
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class SlowDshotTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory()
        directory = Path(cls.temp.name)
        for name in ("common.h", "eeprom.h", "dshot.h", "dshot_slow.h"):
            shutil.copyfile(ROOT / "Inc" / name, directory / name)
        (directory / "main.h").write_text("""
#pragma once
#include <stdint.h>
typedef struct { uint32_t PSC, ARR, EGR, SR; } GPIO_TypeDef;
extern GPIO_TypeDef fake_timer;
""")
        (directory / "targets.h").write_text("""
#pragma once
#define STMICRO
#define MCU_F031
#define DRONECAN_SUPPORT 0
#define CPU_FREQUENCY_MHZ 48
#define LOOP_FREQUENCY_HZ 20000
#define TEN_KHZ_TIMER (&fake_timer)
""")
        (directory / "IO.h").write_text("""
#include "main.h"
extern volatile char inputSet, servoPwm;
extern char dshot;
uint8_t getInputPinState(void);
""")
        for name in ("functions.h", "sounds.h"):
            (directory / name).write_text("")
        (directory / "harness.c").write_text("""
#include "dshot_slow.h"
#include "dshot.h"
GPIO_TypeDef fake_timer;
EEprom_t eepromBuffer;
uint32_t dma_buffer[64];
volatile char inputSet, servoPwm, dshot_telemetry, armed, send_telemetry;
char dshot, forward, play_tone_flag, send_esc_info_flag;
uint8_t running, buffer_padding, dshotcommand, last_dshot_command;
volatile uint16_t newinput;
uint16_t zero_input_count, signaltimeout, battery_voltage;
uint16_t dshot_frametime_low, dshot_frametime_high;
volatile uint32_t commutation_interval;
volatile int16_t degrees_celsius;
int16_t actual_current;
int e_com_time;
static uint8_t pin;
extern uint16_t dshot_goodcounts, dshot_badcounts;
extern uint8_t command_count, last_command, programming_mode;
uint8_t slowDshotReadPin(void) { return pin; }
uint8_t getInputPinState(void) { return 1; }
void saveEEpromSettings(void) {}
#define API __declspec(dllexport)
API void reset(int input_type) {
    eepromBuffer.input_type = input_type;
    eepromBuffer.dir_reversed = 0;
    slowDshotInitTimer();
    slowDshotInit();
    newinput = 0; signaltimeout = 123; zero_input_count = 0;
    dshot_goodcounts = dshot_badcounts = 0;
    command_count = last_command = programming_mode = 0;
    dshotcommand = send_telemetry = 0;
    armed = running = 0;
}
API void sample(int high) { pin = high; slowDshotSamplePin(); }
API int value(void) { return newinput; }
API int good(void) { return dshot_goodcounts; }
API int bad(void) { return dshot_badcounts; }
API int timeout(void) { return signaltimeout; }
API int ready(void) { return inputSet; }
API int zeros(void) { return zero_input_count; }
API int telemetry(void) { return dshot_telemetry || send_telemetry; }
API int reversed(void) { return eepromBuffer.dir_reversed; }
API void arm(void) { armed = 1; }
API int tick(void) { return slowDshotControlTick(); }
API int reload(void) { return fake_timer.ARR; }
""")
        cls.libraries = []
        for hz, mode in ((20000, "DSHOT4"), (40000, "DSHOT8"), (80000, "DSHOT16")):
            library = directory / f"slow_{hz}.dll"
            subprocess.run([
                "clang", "--target=x86_64-pc-windows-msvc", "-fuse-ld=lld",
                "-shared", "-nostdlib", "-O2", "-Wl,/noentry",
                f"-DSLOW_DSHOT_MODE=SLOW_DSHOT_MODE_{mode}", f"-I{directory}",
                str(ROOT / "Src/dshot_slow.c"), str(ROOT / "Src/dshot.c"),
                str(directory / "harness.c"), "-o", str(library),
            ], check=True, capture_output=True, text=True)
            cls.libraries.append((hz, ctypes.CDLL(str(library))))

    @classmethod
    def tearDownClass(cls):
        # Windows holds loaded DLL files until FreeLibrary is called.
        for _, library in cls.libraries:
            ctypes.windll.kernel32.FreeLibrary(ctypes.c_void_p(library._handle))
        cls.temp.cleanup()

    @staticmethod
    def packet(value, telemetry=0):
        payload = (value << 1) | telemetry
        crc = (payload ^ (payload >> 4) ^ (payload >> 8)) & 15
        return (payload << 4) | crc

    @staticmethod
    def feed(library, samples):
        for high in samples:
            library.sample(high)

    def frame(self, library, word, widths=(2, 4), extra_lows=None):
        for bit in range(15, -1, -1):
            width = widths[(word >> bit) & 1]
            self.feed(library, [1] * width + [0] * (5 - width))
            if extra_lows is not None and extra_lows[15 - bit]:
                self.feed(library, [0])

    def test_five_six_and_mixed_length_bits(self):
        for _, library in self.libraries:
            for widths in itertools.product((1, 2), (3, 4)):
                for extra_lows in ([0] * 16, [1] * 16, [0, 1] * 8, [1, 0] * 8):
                    library.reset(1)
                    self.feed(library, [0] * 5)
                    for value in (0, 48, 1000, 2047, 0):
                        self.frame(library, self.packet(value), widths, extra_lows)
                        self.assertEqual(library.value(), value)
                    self.assertEqual(library.good(), 5)
                    self.assertEqual(library.bad(), 0)

    def test_transmitter_phase_and_original_255us_period(self):
        # Time is measured in receiver sample intervals. Both the new 250 us
        # sender (5 intervals) and original 255 us sender (5.1) must decode.
        for _, library in self.libraries:
            for period in (5, 5.1):
                for phase_index in range(100):
                    phase = phase_index / 100
                    for value in (0, 48, 1000, 2047):
                        library.reset(1)
                        self.feed(library, [0] * 5)
                        word = self.packet(value)
                        tick = 0
                        while phase + tick < 16 * period:
                            bit = int((phase + tick) / period)
                            high_time = period * (0.7 if (word >> (15 - bit)) & 1 else 0.3)
                            self.feed(library, [phase + tick - bit * period < high_time])
                            tick += 1
                        self.assertEqual(library.good(), 1, (period, phase, value))
                        self.assertEqual(library.bad(), 0, (period, phase, value))
                        self.assertEqual(library.value(), value)

    def test_throttle_all_widths_and_back_to_back(self):
        for _, library in self.libraries:
            for widths in itertools.product((1, 2), (3, 4)):
                library.reset(1)
                self.feed(library, [0] * 5)
                for value in (0, 48, 1000, 2047, 0):
                    self.frame(library, self.packet(value, telemetry=1), widths)
                    self.assertEqual(library.value(), value)
                self.assertEqual(library.good(), 5)
                self.assertEqual(library.timeout(), 0)
                self.assertEqual(library.ready(), 1)
                self.assertEqual(library.telemetry(), 0)

    def test_crc_failure_requires_idle_and_preserves_timeout(self):
        for _, library in self.libraries:
            library.reset(1)
            self.feed(library, [0] * 5)
            self.frame(library, self.packet(1000) ^ 1)
            self.assertEqual(library.good(), 0)
            self.assertEqual(library.bad(), 1)
            self.assertEqual(library.timeout(), 123)
            self.frame(library, self.packet(1000))
            self.assertEqual(library.good(), 0)
            self.feed(library, [0] * 5)
            self.frame(library, self.packet(1000))
            self.assertEqual(library.good(), 1)

    def test_malformed_patterns_and_idle_recovery(self):
        for _, library in self.libraries:
            for pattern in ([1, 0, 1, 0, 0], [1] * 5):
                library.reset(1)
                self.feed(library, [0] * 5 + pattern)
                self.assertEqual(library.good(), 0)
                self.assertEqual(library.bad(), 1)
                self.assertEqual(library.timeout(), 123)
                self.feed(library, [0] * 5)
                self.frame(library, self.packet(1000))
                self.assertEqual(library.good(), 1)
            # One extra low is allowed; a second extra low rejects the packet.
            library.reset(1)
            self.feed(library, [0] * 5 + [1, 1, 0, 0, 0] + [0])
            self.assertEqual(library.bad(), 0)
            self.feed(library, [0])
            self.assertEqual(library.bad(), 1)
            self.assertEqual(library.ready(), 0)
            self.assertEqual(library.timeout(), 123)
            self.feed(library, [0] * 5)
            self.frame(library, self.packet(1000))
            self.assertEqual(library.good(), 1)

    def test_requires_explicit_eeprom_dshot(self):
        for _, library in self.libraries:
            for input_type in (0, 2, 3, 4, 5):
                library.reset(input_type)
                self.feed(library, [0] * 5)
                self.frame(library, self.packet(1000))
                self.assertEqual(library.good(), 0)
                self.assertEqual(library.ready(), 0)

    def test_startup_idle_and_arming_zero_count(self):
        for _, library in self.libraries:
            library.reset(1)
            self.frame(library, self.packet(1000))
            self.assertEqual(library.good(), 0)
            self.feed(library, [0] * 5)
            for _ in range(31):
                self.frame(library, self.packet(0))
            self.assertEqual(library.zeros(), 31)

    def test_commands_reuse_existing_decoder(self):
        for _, library in self.libraries:
            library.reset(1)
            library.arm()
            self.feed(library, [0] * 5)
            for _ in range(6):
                self.frame(library, self.packet(8))
            self.assertEqual(library.reversed(), 1)

    def test_exact_timer_and_control_divider(self):
        for hz, library in self.libraries:
            library.reset(1)
            self.assertEqual(library.reload(), 2000000 // hz - 1)
            ticks = [library.tick() for _ in range(hz // 1000)]
            self.assertEqual(sum(ticks), 20)


if __name__ == "__main__":
    unittest.main()
