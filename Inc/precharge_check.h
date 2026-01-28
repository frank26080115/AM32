#include "main.h"

#ifndef PRECHARGE_CHECK_H_
#define PRECHARGE_CHECK_H_

extern void precharge_require(void);
extern void precharge_stage2(void);
extern void precharge_poll(char force);
extern void precharge_static_test(void);

#endif
