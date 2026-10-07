#ifndef INC_DSHOT_SLOW_H_
#define INC_DSHOT_SLOW_H_

#include <stdint.h>
#include "main.h"
#include "targets.h"
#include "common.h"

#define SLOW_DSHOT_MODE_DSHOT4      4
#define SLOW_DSHOT_MODE_DSHOT8      8
#define SLOW_DSHOT_MODE_DSHOT16     16

// Choose ONLY ONE of these three modes; match the ELRS output selection.
// DSHOT4: 20 kHz sampling, DSHOT8: 40 kHz, DSHOT16: 80 kHz.
#ifndef SLOW_DSHOT_MODE
#define SLOW_DSHOT_MODE     SLOW_DSHOT_MODE_DSHOT4
//#define SLOW_DSHOT_MODE     SLOW_DSHOT_MODE_DSHOT8
//#define SLOW_DSHOT_MODE     SLOW_DSHOT_MODE_DSHOT16
#endif

#ifdef SPECIAL_BUILD_SLOW_DSHOT
// Derived frequency for calculations; select a mode above instead of editing it.
// The special-build timer uses exact 20/40/80 kHz, unlike the original
// nominal 20 kHz timer (19,607.843 Hz).
#ifdef SLOW_DSHOT_SAMPLE_HZ
#error "Select SLOW_DSHOT_MODE; SLOW_DSHOT_SAMPLE_HZ is derived automatically"
#endif
#if SLOW_DSHOT_MODE == SLOW_DSHOT_MODE_DSHOT4
#define SLOW_DSHOT_SAMPLE_HZ 20000
#elif SLOW_DSHOT_MODE == SLOW_DSHOT_MODE_DSHOT8
#define SLOW_DSHOT_SAMPLE_HZ 40000
#elif SLOW_DSHOT_MODE == SLOW_DSHOT_MODE_DSHOT16
#define SLOW_DSHOT_SAMPLE_HZ 80000
#else
#error "SLOW_DSHOT_MODE must be SLOW_DSHOT_MODE_DSHOT4, SLOW_DSHOT_MODE_DSHOT8, or SLOW_DSHOT_MODE_DSHOT16"
#endif
#endif

#ifdef SPECIAL_BUILD_SLOW_DSHOT
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
