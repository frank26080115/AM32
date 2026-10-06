#ifndef INC_DSHOT_SLOW_H_
#define INC_DSHOT_SLOW_H_

#include <stdint.h>
#include "main.h"
#include "targets.h"
#include "common.h"

#ifdef SPECIAL_BUILD_SLOW_DSHOT
#if SLOW_DSHOT_SAMPLE_HZ != 20000 && SLOW_DSHOT_SAMPLE_HZ != 40000 && SLOW_DSHOT_SAMPLE_HZ != 80000
#error "Slow DShot sampling must be 20, 40, or 80 kHz"
#endif
#if SLOW_DSHOT_SAMPLE_HZ % LOOP_FREQUENCY_HZ != 0
#error "Slow DShot sampling must be an integer multiple of the control loop rate"
#endif
#if defined(USE_ADC_INPUT) || defined(USE_CRSF_INPUT) || defined(FIXED_DUTY_MODE) || defined(FIXED_SPEED_MODE)
#error "Slow DShot requires the DShot input path"
#endif

void slowDshotInit(void);
void slowDshotInitTimer(void);
void slowDshotInitPin(void);
uint8_t slowDshotReadPin(void);
void slowDshotSamplePin(void);
uint8_t slowDshotControlTick(void);
extern volatile uint8_t slow_dshot_input_ready;
#endif

#endif
