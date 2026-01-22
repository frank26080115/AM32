#include "sounds.h"
#include "common.h"
#include "eeprom.h"
#include "functions.h"
#include "peripherals.h"
#include "phaseouts.h"
#include "targets.h"

#define PRECHARGE_TONE_VOLUME            4
// tune this for conditions
// the unit is the EEMPROM volume number * 2, so if the configurator sets 1, this value is 2
// note: the 39ohm resistor is typically rated 1W so we only want 150mA, 3W for 500ms is ok
// warning: absolutely do not go above 20 for this value

#define PRECHARGE_TONE_FREQ_PRESCALER    5
// lower number = higher pitched tone
// the default startup tone ends at prescaler 25

#define PRECHARGE_TONE_DURATION_MS       200
// we need to play the tone long enough to drain the capacitor but also not burn out the precharge resistor
// if the current draw is low enough, this can be longer for bigger capacitors
// if the current draw is high, then keep this short, and hope that the capacitors drain faster
// estimates say pushing a 1W resistor to 3W for 500ms is likely safe

#define PRECHARGE_DROP_THRESHOLD         58
// threshold for pass or fail the precharge check
// if the battery voltage drops this much due to the tone, then the test fails
// unit is decivolts, volts*10, example: 58 means 5.8 volts, which is 39ohms and 150mA (this is under 1W)

extern uint8_t beep_volume;
extern char armed;
extern uint16_t battery_voltage;
extern uint16_t ADC_raw_volts;

char precharge_state = 0;
uint16_t precharge_max_batt = 0;

char precharge_check(void)
{
    // return 1 if not fully powered
    char ret = 0;

    #if 0
    if (precharge_state == 2) {
        // if already checked, return immediately
        return ret;
    }
    #endif

    __disable_irq();

    // for wait for capacitor to charge
    if (precharge_max_batt == 0) {
        precharge_max_batt = precharge_wait_rise(50);
    }

    // start playing tone
    SET_DUTY_CYCLE_ALL(PRECHARGE_TONE_VOLUME);
    SET_AUTO_RELOAD_PWM(TIM1_AUTORELOAD);
    RELOAD_WATCHDOG_COUNTER();
    SET_PRESCALER_PWM(PRECHARGE_TONE_FREQ_PRESCALER);
    setCaptureCompare();
    comStep(6);
    for (uint8_t i = 0; i < PRECHARGE_TONE_DURATION_MS; i++) {
        RELOAD_WATCHDOG_COUNTER();
        delayMillis(1);
        uint16_t bv = precharge_adc();
        if (bv < (precharge_max_batt - PRECHARGE_DROP_THRESHOLD)) {
            // quit early if voltage drops too much
            ret = 1;
            break;
        }
    }
    allOff();
    SET_PRESCALER_PWM(0);
    signaltimeout = 0;
    SET_AUTO_RELOAD_PWM(TIMER1_MAX_ARR);
    __enable_irq();

    SET_DUTY_CYCLE_ALL(beep_volume); // restore for normal operation later

    if (ret) {
        // force a disarm
        armed = 0;
        precharge_state = 1; // indicate check finished and failed
    }
    else {
        precharge_state = 2; // indicate check finished and passed
    }

    return ret;
}

uint16_t precharge_adc(void)
{
    // returns filtered battery voltage in decivolts

    char data = 0;

    // check if DMA has new data from ADC
    // NOTE: some of the code don't even check the flag, and just assume the ADC is faster than 1 KHz

    #if defined(STMICRO)
        #if defined(MCU_F051)
        if (LL_DMA_IsActiveFlag_TC1(DMA1) != 0) {
            LL_DMA_ClearFlag_GI1(DMA1);
        #elif defined(MCU_F031)
        if (LL_DMA_IsActiveFlag_TC2(DMA1) != 0) {
            LL_DMA_ClearFlag_GI2(DMA1);
        #elif defined(MCU_G071)
        if (LL_DMA_IsActiveFlag_TC2(DMA1) != 0) {
            LL_DMA_ClearFlag_GI2(DMA1);
        #elif defined(MCU_G031)
        if (LL_DMA_IsActiveFlag_TC2(DMA1) != 0) {
            LL_DMA_ClearFlag_GI2(DMA1);
        #elif defined(MCU_G431)
        if (LL_DMA_IsActiveFlag_TC2(DMA1) != 0) {
            LL_DMA_ClearFlag_GI2(DMA1);
        #elif defined(MCU_L431)
        if (LL_DMA_IsActiveFlag_TC1(DMA1) != 0) {
            LL_DMA_ClearFlag_GI1(DMA1);
        #else
        {
        #endif
            ADC_DMA_Callback();
            LL_ADC_REG_StartConversion(ADC1);
            data = 1;
        }
    #elif defined(MCU_GDE23)
        // don't check DMA, assume ready
        ADC_DMA_Callback();
        adc_software_trigger_enable(ADC_REGULAR_CHANNEL);
        data = 1;
    #elif defined(ARTERY)
        if (dma_flag_get(DMA1_FDT1_FLAG) == SET) {
            DMA1->clr = DMA1_GL1_FLAG;
            #ifdef USE_ADC
            ADC_DMA_Callback();
            adc_ordinary_software_trigger_enable(ADC1, TRUE);
            data = 1;
            #endif
        }
    #elif defined(WCH)
        if(DMA_GetITStatus(DMA1_IT_TC1))
        {
            DMA_ClearFlag(DMA1_IT_TC1|DMA1_IT_HT1);
            ADC_DMA_Callback( );
            startADCConversion()
            data = 1;
        }
    #else
        ADC_DMA_Callback();
        data = 1;
    #endif
    if (data)
    {
        uint32_t current_voltage = ((ADC_raw_volts * 3300 / 4095 * VOLTAGE_DIVIDER) / 100);
        battery_voltage = (battery_voltage == 0) ? current_voltage : (((3 * battery_voltage) + current_voltage) >> 2);
    }
    return battery_voltage;
}

uint16_t precharge_wait_rise(uint16_t t)
{
    // returns maximum battery voltage in decivolts

    uint16_t prev_v = 0;
    uint16_t max_v = 0;
    uint32_t t_rem = t;
    while (t_rem--) {
        RELOAD_WATCHDOG_COUNTER();
        delayMillis(1);
        uint16_t bv = precharge_adc();
        max_v = (bv > max_v) ? bv : max_v;
        if (bv > prev_v) {
            t_rem = t; // reset timer
        }
        prev_v = bv;
    }
    return max_v;
}
