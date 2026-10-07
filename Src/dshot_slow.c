#include "IO.h"
#include "dshot.h"
#include "dshot_slow.h"

#ifdef SPECIAL_BUILD_SLOW_DSHOT

enum sample_state { WAIT_IDLE, WAIT_EDGE, RECEIVE_BITS, WAIT_NEXT_BIT };
static enum sample_state state;
static uint8_t enabled;
static uint8_t low_samples;
static uint8_t previous_high;
static uint8_t bit_index;
static uint8_t sample_index;
static uint8_t high_samples;
static uint8_t saw_low;
static uint16_t control_divider;
volatile uint8_t slow_dshot_input_ready;
extern uint16_t dshot_goodcounts;
extern uint16_t dshot_badcounts;
extern volatile char inputSet;
extern char dshot;
extern volatile char servoPwm;

void slowDshotInit(void)
{
    enabled = eepromBuffer.input_type == DSHOT_IN;
    dshot = enabled;
    servoPwm = 0;
    dshot_telemetry = 0;
    EDT_ARM_ENABLE = 0;
    EDT_ARMED = 1;
    inputSet = 0; // Only a CRC-valid packet establishes an input signal.
    state = WAIT_IDLE;
    low_samples = previous_high = bit_index = sample_index = 0;
    slow_dshot_input_ready = 0;
    // Synthetic timestamps use five ticks per bit (frame span 76..79).
    dshot_frametime_low = 75;
    dshot_frametime_high = 80;
}

uint8_t slowDshotControlTick(void)
{
    if (++control_divider == SLOW_DSHOT_SAMPLE_HZ / LOOP_FREQUENCY_HZ) {
        control_divider = 0;
        return 1;
    }
    return 0;
}

static void rejectPacket(void)
{
    dshot_badcounts++;
    state = WAIT_IDLE;
}

static void beginBit(void)
{
    state = RECEIVE_BITS;
    sample_index = high_samples = 1; // Rising edge is sample one.
    saw_low = 0;
}

void slowDshotSamplePin(void)
{
    if (!enabled) {
        return;
    }
    const uint8_t high = slowDshotReadPin() != 0;
    if (high) {
        low_samples = 0;
    } else if (low_samples < 5) {
        low_samples++;
    }

    if (state == WAIT_IDLE) {
        if (low_samples == 5) {
            state = WAIT_EDGE;
        }
    } else if (state == WAIT_EDGE) {
        if (high && !previous_high) {
            bit_index = 0;
            beginBit();
        }
    } else if (state == WAIT_NEXT_BIT) {
        if (high) {
            beginBit(); // Resynchronize every bit after its mandatory low sample.
        } else if (sample_index == 0) {
            sample_index = 1; // Allow one extra low: this bit lasts six samples.
        } else {
            rejectPacket(); // A seventh low sample cannot belong to this bit.
        }
    } else {
        if (high) {
            if (saw_low) { // Only a contiguous high prefix is accepted.
                rejectPacket();
                previous_high = high;
                return;
            }
            high_samples++;
        } else {
            saw_low = 1;
        }
        if (++sample_index == 5) {
            if (high || high_samples == 0 || high_samples > 4) {
                rejectPacket();
            } else {
                dma_buffer[2 * bit_index] = 5 * bit_index;
                dma_buffer[2 * bit_index + 1] = 5 * bit_index + high_samples;
                sample_index = 0;
                state = WAIT_NEXT_BIT;
                if (++bit_index == 16) {
                    const uint16_t good_before = dshot_goodcounts;
                    computeDshotDMA(); // Reuse the existing CRC/command/throttle decoder.
                    if (dshot_goodcounts != good_before) {
                        inputSet = 1;
                        slow_dshot_input_ready = 1;
                        if (!armed) {
                            if (newinput == 0) {
                                if (zero_input_count < UINT16_MAX) {
                                    zero_input_count++;
                                }
                            } else {
                                zero_input_count = 0;
                            }
                        }
#if defined(NEED_INPUT_READY) || defined(NXP)
                        input_ready = 1;
#endif
                        state = WAIT_EDGE; // No idle gap required after a valid packet.
                    } else {
                        state = WAIT_IDLE;
                    }
                }
            }
        }
    }
    previous_high = high;
}

void slowDshotInitTimer(void)
{
    control_divider = 0;
#if defined(STMICRO)
    TEN_KHZ_TIMER->PSC = CPU_FREQUENCY_MHZ / 2 - 1;
    TEN_KHZ_TIMER->ARR = 2000000 / SLOW_DSHOT_SAMPLE_HZ - 1;
    TEN_KHZ_TIMER->EGR = 1;
    TEN_KHZ_TIMER->SR = 0;
#elif defined(ARTERY)
    TEN_KHZ_TIMER->div = CPU_FREQUENCY_MHZ / 2 - 1;
    TEN_KHZ_TIMER->pr = 2000000 / SLOW_DSHOT_SAMPLE_HZ - 1;
    TEN_KHZ_TIMER->swevt = TMR_OVERFLOW_SWTRIG;
    TEN_KHZ_TIMER->ists = 0;
#elif defined(GIGADEVICES)
    TIMER_PSC(TEN_KHZ_TIMER) = CPU_FREQUENCY_MHZ / 2 - 1;
    TIMER_CAR(TEN_KHZ_TIMER) = 2000000 / SLOW_DSHOT_SAMPLE_HZ - 1;
    timer_event_software_generate(TEN_KHZ_TIMER, TIMER_EVENT_SRC_UPG);
    TIMER_INTF(TEN_KHZ_TIMER) = 0;
#elif defined(NXP)
    modifyReg32(&SYSCON->CLKUNLOCK, SYSCON_CLKUNLOCK_UNLOCK(1), 0);
    modifyReg32(&MRCC0->MRCC_LPTMR0_CLKSEL, MRCC_MRCC_LPTMR0_CLKSEL_MUX_MASK,
                MRCC_MRCC_LPTMR0_CLKSEL_MUX(0)); // FRO_12M, divider /1.
    modifyReg32(&SYSCON->CLKUNLOCK, 0, SYSCON_CLKUNLOCK_UNLOCK(1));
    LPTMR0->CMR = 12000000 / SLOW_DSHOT_SAMPLE_HZ - 1;
#elif defined(WCH)
    SysTick->CTLR = 0;
    SysTick->CNT = 0;
    SysTick->CMP = SystemCoreClock / SLOW_DSHOT_SAMPLE_HZ - 1;
    SysTick->SR = 0;
    SysTick->CTLR = 0xF;
#else
#error "Slow DShot timer configuration is missing for this processor"
#endif
}
#endif
