# Slow DShot special build

`SPECIAL_BUILD_SLOW_DSHOT` in `Inc/common.h` selects fixed-interval GPIO
sampling instead of timer edge capture/DMA. Comment out that definition to
restore the standard input implementation. Configure EEPROM input method to
**DShot (`DSHOT_IN`, value 1)**. Auto detection, servo input, EDT-arm input and
bidirectional DShot telemetry are not supported by this special build.

`SLOW_DSHOT_SAMPLE_HZ` defaults to 20000 and can be overridden to 40000 or
80000, including with a compiler definition. The timer is programmed for the
exact nominal rate; the existing control routine runs every 1, 2 or 4 sampling
interrupts respectively. Targets with a lower configured control rate retain
that rate through an integer divider. Standard builds retain their original
timer settings.

| Sampling | Bit rate (five samples/bit) | Bit duration | Packet duration |
|---|---|---|---|
| 20 kHz | 4 kbit/s | 250 us | 4 ms |
| 40 kHz | 8 kbit/s | 125 us | 2 ms |
| 80 kHz | 16 kbit/s | 62.5 us | 1 ms |

For ESP32 RMT, transmit ordinary, non-inverted DShot bits MSB first, with the
usual 12-bit payload and four-bit XOR checksum. Only pulse timing changes.
Start with a constant bit period of five sampling intervals, a zero high time
of 37.5% of the bit period and a one high time of 75%:

| Sampling | Zero high / low | One high / low |
|---|---|---|
| 20 kHz | 93.75 / 156.25 us | 187.5 / 62.5 us |
| 40 kHz | 46.875 / 78.125 us | 93.75 / 31.25 us |
| 80 kHz | 23.4375 / 39.0625 us | 46.875 / 15.625 us |

Choose an RMT tick that represents these timings sufficiently accurately.
Allow at least five observed low samples before the first packet and after
errors; six sampling intervals of low time provide margin for asynchronous
sampling. Consecutive CRC-valid packets need no extra idle gap. Match the
transmitter bit period to the receiver's actual clock; both oscillators have
tolerance, and phase drift across a packet reduces the sampling margin.

The accepted sample groups are `10000`, `11000`, `11100`, and `11110`.
The first two encode zero and the latter two encode one. A low first sample,
high fifth sample or a second high pulse within a bit rejects the packet.
The sampler writes synthetic edge timestamps into `dma_buffer` and calls the
existing `computeDshotDMA()` for checksum, throttle and command processing.
Only CRC-valid packets reset the signal timeout and establish input validity.

The GPIO read is normalized in each MCU's `IO.c`; the packet state machine is
shared. The Makefile discovers `Src/dshot_slow.c` automatically. IDE projects
with explicit source lists must add that file.

Sampling shares the control ISR. Blocking commutation polling, arming tones,
delays, EEPROM writes and interrupt masking can cause missed samples. Higher
priority interrupts can introduce jitter. This implementation does not promise
gap-free sampling; measure it on the target with the intended motor workload.
For ordinary 20 kHz targets the special build corrects the previous approximately
19.608 kHz timer rate, changing tick-based timing by about 2%. It does not alter
the existing PID divider's `>` comparison.

Host regression tests use the actual sampler and decoder, with mocked GPIO and
timer registers: `python tests/test_dshot_slow.py` (Windows, clang/lld).
