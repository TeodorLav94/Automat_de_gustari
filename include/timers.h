#ifndef TIMERS_H
#define TIMERS_H

#include <stdint.h>

extern volatile uint32_t systicks;

#define SYSTICKS_PASSED(last, delta) ((uint32_t)(systicks - (last)) >= (delta))

void Timer0_init_ctc(void);    // systicks: 1 ms tick cu Timer0
void Timer1_init_pwm(void);    // PWM servo: OC1A, 50 Hz

#endif
