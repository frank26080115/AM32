#include "precharge_check.h"
#include "sounds.h"
#include "common.h"
#include "eeprom.h"
#include "functions.h"
#include "peripherals.h"
#include "phaseouts.h"
#include "targets.h"

//#define PRECHARGE_DROP_THRESHOLD_RUNNING   400//580
// threshold for pass or fail the precharge check
// if the battery voltage drops this much due to running motor, then the test fails
// unit is centivolts, volts*100, example: 580 means 5.8 volts, which is 39ohms and 150mA (this is under 1W)

#define PRECHARGE_DROP_THRESHOLD_TONE      25
// if the battery voltage drops this much due to static tone, then the test fails

//#define PRECHARGE_CURRENT_THRESHOLD    10
// in centiamps, so 10 means 0.1A
// when the current measured exceeds this value, the voltage drop is checked

#define PRECHARGE_TEST_PASSED_TIME     500
// number of milliseconds that the test must pass before the test never happens again

#define PRECHARGE_VOLTAGE_SETTLE_TIME  200
// number of milliseconds that the voltage must settle (or start declining) before testing can happen

#define PRECHARGE_TONE_RANDOM_DELAY
// if multiple ESCs are used behind one switch, it might be good so they don't all perform the test at the same time

extern char armed;
extern uint8_t running;
extern uint16_t input;
extern uint16_t beep_volume;
extern uint16_t ADC_raw_volts;
extern uint16_t ADC_raw_current;
extern uint16_t VOLTAGE_DIVIDER;
extern uint16_t adjusted_duty_cycle;
extern uint16_t tim1_arr;

extern void ADC_DMA_Callback(void);
extern void setCaptureCompare(void);

char prechg_check_stage = 0;
char prechg_tripped = 0;
uint32_t prechg_bv_flt_heavy = 0;
uint32_t prechg_bv_temp_max = 0;
uint16_t prechg_bv_settled_count = 0;
uint32_t prechg_bv_settled = 0;
//uint32_t prechg_bv_flt_medium = 0;
uint32_t prechg_bv_flt_light  = 0;
uint32_t prechg_cur_flt_heavy = 0;
uint32_t prechg_cur_settled = 0;
uint32_t prechg_passed_cnt = 0;

void precharge_require(void)
{
    #if !defined(PRECHARGE_DROP_THRESHOLD_RUNNING) && !defined(PRECHARGE_DROP_THRESHOLD_TONE)
    return;
    #endif
    prechg_check_stage = 1;
    prechg_passed_cnt = 0;
    prechg_tripped = 0;
}

void precharge_stage2(void)
{
    if (prechg_check_stage == 0) {
        return;
    }
    if (prechg_check_stage != 2) {
        //prechg_passed_cnt = 0;
        prechg_tripped = 0;
    }
    #if defined(PRECHARGE_DROP_THRESHOLD_RUNNING)
    prechg_check_stage = 2;
    #else
    prechg_check_stage = 0;
    #endif
}

void precharge_static_test_p(uint32_t duration, uint32_t volume, uint32_t prescaler, uint8_t step)
{
    if (prechg_check_stage == 0) {
        // this means check is not needed
        return;
    }
    __disable_irq();
    RELOAD_WATCHDOG_COUNTER();
    SET_DUTY_CYCLE_ALL(volume);
    SET_AUTO_RELOAD_PWM(TIM1_AUTORELOAD);
    setCaptureCompare();
    SET_PRESCALER_PWM(prescaler);
    step = (step % 6) + 1;
    comStep(step);
    for (uint32_t i = 0; i < duration; i++)
    {
        RELOAD_WATCHDOG_COUNTER();
        delayMicros(1000);
        precharge_poll(0);
    }
    allOff();
    SET_PRESCALER_PWM(0);
    SET_AUTO_RELOAD_PWM(TIMER1_MAX_ARR);
    SET_DUTY_CYCLE_ALL(beep_volume);
    __enable_irq();
}

void precharge_static_test(void)
{
    #ifdef PRECHARGE_DROP_THRESHOLD_TONE

    if (prechg_check_stage == 0) {
        // this means check is not needed
        return;
    }

    // battery voltage must settle before this works
    while (prechg_bv_settled == 0) {
        RELOAD_WATCHDOG_COUNTER();
        delayMicros(1000);
        precharge_poll(0);
    }
    // this is done here so that maybe the volume can be adjusted later according to input voltage

    #ifdef PRECHARGE_TONE_RANDOM_DELAY
    // random delay so that multiple ESCs don't poll at the same time
    uint8_t rand_delay = prng8(ADC_raw_volts) & 0x07;
    for (uint8_t i = 0; i < rand_delay; i++) {
        RELOAD_WATCHDOG_COUNTER();
        delayMicros(1000);
        precharge_poll(0);
    }
    #endif

    precharge_static_test_p(200,
        TIM1_AUTORELOAD / 4, 20,
        ADC_raw_volts & 0x3F);

    #endif
}

void precharge_poll(char force)
{
    if (prechg_check_stage == 0) {
        // this means check is not needed
        return;
    }

    static uint32_t last_time = 0;
    volatile uint32_t curr_time;
    // utility timer counts microseconds
    #if defined(STMICRO)
        curr_time = UTILITY_TIMER->CNT;
    #elif defined(GIGADEVICES)
        curr_time = TIMER_CNT(UTILITY_TIMER);
    #elif defined(ARTERY)
        curr_time = UTILITY_TIMER->cval;
    #elif defined(WCH)
        curr_time = UTILITY_TIMER->CNT>>1;
    #else
        #error unsupported MCU
    #endif

    // we want to execute only once every millisecond
    if ((curr_time - last_time) < 1000 && force == 0) {
        return; // not time yet
    }
    last_time = curr_time;

    if (force == 0)
    {
        // this means running from a tight loop, so we need to check the ADC
        #if defined(STMICRO)
            ADC_DMA_Callback();
            LL_ADC_REG_StartConversion(ADC1);
        #elif defined(MCU_GDE23)
            // don't check DMA, assume ready
            ADC_DMA_Callback();
            adc_software_trigger_enable(ADC_REGULAR_CHANNEL);
        #elif defined(ARTERY)
            ADC_DMA_Callback();
            adc_ordinary_software_trigger_enable(ADC1, TRUE);
        #elif defined(WCH)
            DMA_ClearFlag(DMA1_IT_TC1|DMA1_IT_HT1);
            ADC_DMA_Callback();
            startADCConversion();
        #else
            ADC_DMA_Callback();
        #endif
    }

    if (ADC_raw_volts == 0) {
        // no ADC result? do not do anything
        return;
    }

    uint32_t converted_voltage = ((ADC_raw_volts * 3300 / 4095 * VOLTAGE_DIVIDER) / 100);
    prechg_bv_flt_heavy  = (prechg_bv_flt_heavy  == 0) ? converted_voltage : (((31 * prechg_bv_flt_heavy)  + converted_voltage) >> 5);
    //prechg_bv_flt_medium = (prechg_bv_flt_medium == 0) ? converted_voltage : ((( 7 * prechg_bv_flt_medium) + converted_voltage) >> 3);
    prechg_bv_flt_light  = (prechg_bv_flt_light  == 0) ? converted_voltage : ((( 3 * prechg_bv_flt_light)  + converted_voltage) >> 2);

    #ifdef PRECHARGE_CURRENT_THRESHOLD
    uint32_t converted_current = ((ADC_raw_current * 3300 / 41) - (CURRENT_OFFSET * 100)) / (MILLIVOLT_PER_AMP);
    prechg_cur_flt_heavy = ((15 * prechg_cur_flt_heavy) + converted_current) >> 4;
    #endif

    // track maximum voltage but heavily filtered, wait for settling before the value can be used
    if (prechg_bv_flt_heavy > prechg_bv_temp_max) {
        // if a new max is reached, then the voltage has not settled, the capacitor is likely still charging
        prechg_bv_temp_max = prechg_bv_flt_heavy;
        prechg_bv_settled_count = 0;
    }
    else {
        // if enough time passed
        if (prechg_bv_settled_count == PRECHARGE_VOLTAGE_SETTLE_TIME) {
            if (prechg_bv_settled == 0) {
                // if first time
                prechg_bv_settled = prechg_bv_temp_max;
                prechg_cur_settled = prechg_cur_flt_heavy;
                prechg_bv_temp_max = 0;
            }
            else if (prechg_bv_flt_heavy > prechg_bv_settled) {
                // if not the first time, perhaps the switch closed completely
                prechg_bv_settled = prechg_bv_flt_heavy;
                prechg_bv_temp_max = 0;
            }
            prechg_bv_settled_count++;
        }
        else if (prechg_bv_settled_count < PRECHARGE_VOLTAGE_SETTLE_TIME) {
            prechg_bv_settled_count++;
        }
    }

    if (prechg_bv_settled != 0) // voltage settled and we want to check
    {
        char motor_running = 0;

        #ifdef PRECHARGE_CURRENT_THRESHOLD
        // if we are drawing enough current, then the motor is running
        motor_running |= (prechg_cur_flt_heavy > prechg_cur_settled && (prechg_cur_flt_heavy - prechg_cur_settled) > PRECHARGE_CURRENT_THRESHOLD);
        #endif
        if (armed && running && input > 50) {
            #if defined(PRECHARGE_DROP_THRESHOLD_RUNNING)
                prechg_check_stage = 2;
            #else
                prechg_check_stage = 0;
                return;
            #endif
        }
        if (prechg_check_stage > 1) {
            // if the throttle is above 12%, consider the motor to be running
            motor_running |= (adjusted_duty_cycle > (tim1_arr / 8) || adjusted_duty_cycle > (TIMER1_MAX_ARR / 8) || adjusted_duty_cycle > (TIM1_AUTORELOAD / 8));
        }

        if (motor_running || prechg_check_stage == 1)
        {
            uint32_t drop_thresh = (prechg_check_stage == 1) ?
                #if defined(PRECHARGE_DROP_THRESHOLD_RUNNING) && defined(PRECHARGE_DROP_THRESHOLD_TONE)
                    PRECHARGE_DROP_THRESHOLD_TONE : PRECHARGE_DROP_THRESHOLD_RUNNING;
                #elif defined(PRECHARGE_DROP_THRESHOLD_TONE)
                    PRECHARGE_DROP_THRESHOLD_TONE : 0;
                #elif defined(PRECHARGE_DROP_THRESHOLD_RUNNING)
                    0 : PRECHARGE_DROP_THRESHOLD_RUNNING;
                #else
                    0 : 0;
                #endif

            if (prechg_bv_flt_light < (prechg_bv_settled - drop_thresh) && drop_thresh > 0)
            {
                // test failed

                __disable_irq();
                RELOAD_WATCHDOG_COUNTER();

                prechg_tripped = 1;
                armed = 0;

                // actually turn off all motors
                SET_DUTY_CYCLE_ALL(0);
                allOff();

                prechg_check_stage = 0; // prevents recursion in next delayMillis call
                // indicate to user
                #ifdef USE_LED_STRIP
                    delayMicros(1000);
                    send_LED_RGB(0, 0, 128);
                    delayMillis(500); // let the LED show for a bit before reset
                #endif
                #ifdef USE_RGB_LED
                    setIndividualRGBLed(0,0,1);
                    delayMillis(500); // let the LED show for a bit before reset
                #endif
                NVIC_SystemReset();
            }
            else
            {
                prechg_tripped = 0;
                if (motor_running) {
                    // only count the time if the motor is running
                    prechg_passed_cnt++;
                    // if we have passed the test for a long period, do not run the test anymore, preventing accidental false positives
                    if (prechg_passed_cnt >= PRECHARGE_TEST_PASSED_TIME) {
                        prechg_check_stage = 0;
                    }
                }
            }
        }
    }
}
