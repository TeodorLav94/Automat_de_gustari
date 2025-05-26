#include <avr/io.h>
#include <avr/interrupt.h>
#include "timers.h"

volatile uint32_t systicks = 0;

void Timer0_init_ctc(void) {
    // Timer0 in CTC mode, prescaler 64, OCR0A = 249 → 1ms tick
    TCCR0A = (1 << WGM01);                  // CTC mode
    TCCR0B = (1 << CS01) | (1 << CS00);     // prescaler = 64
    OCR0A = 249;                            // 16MHz / 64 / 1000 - 1
    TIMSK0 |= (1 << OCIE0A);                // enable compare match A interrupt
}

ISR(TIMER0_COMPA_vect) {
    systicks++;
}

void Timer1_init_pwm(void) {
    DDRB |= (1 << PB3);  // D11 as output

    TCCR1A = (1 << COM1A1) | (1 << WGM11);               // Non-inverting, mode 14
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);  // Prescaler = 8
    ICR1 = 40000;   // f = 50 Hz (20ms)
    OCR1A = 2000;   // ~1 ms pulse (0°)
}