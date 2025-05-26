#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>
#include <util/delay.h>

#include "timers.h"
#include "usart.h"

// ================== Configurări ==================

#define PM_BAUD 28800

// LED-uri
#define LED_BLUE_PIN   PD5  // D5
#define LED_GREEN_PIN  PB4  // D12
#define LED_RED_PIN    PB0  // D8

// Butoane
#define BUTTON_LEFT_PIN    PD6  // D6 - polling
#define BUTTON_RIGHT_PIN   PD2  // D2 - INT0
#define BUTTON_RANDOM_PIN  PB5  // D13 - polling nou

// Buzzer
#define BUZZER_PIN     PD7  // D7

// Servomotoare
#define SERVO1_PIN     PB1  // D9 - OC1A
#define SERVO2_PIN     PB2  // D10 - OC1B
#define SERVO3_PIN     PD3  // D3 - software manual

#define LED_ANIMATION_TIME 2000  // în ms

volatile uint8_t current_state = 0;
volatile uint8_t right_button_flag = 0;

// ================== Inițializări ==================

void GPIO_init(void) {
    DDRD |= (1 << LED_BLUE_PIN);  
    DDRB |= (1 << LED_GREEN_PIN) | (1 << LED_RED_PIN);

    DDRD |= (1 << BUZZER_PIN);
    PORTD &= ~(1 << BUZZER_PIN);

    DDRD &= ~(1 << BUTTON_LEFT_PIN);
    PORTD |= (1 << BUTTON_LEFT_PIN);

    DDRD &= ~(1 << BUTTON_RIGHT_PIN);
    PORTD |= (1 << BUTTON_RIGHT_PIN);

    DDRB &= ~(1 << BUTTON_RANDOM_PIN);
    PORTB |= (1 << BUTTON_RANDOM_PIN);

    DDRB |= (1 << SERVO1_PIN) | (1 << SERVO2_PIN);
    DDRD |= (1 << SERVO3_PIN);  
}

void interrupts_init(void) {
    EIMSK |= (1 << INT0);
    EICRA |= (1 << ISC01);  
}

void update_leds(uint8_t state) {
    PORTD &= ~(1 << LED_BLUE_PIN);
    PORTB &= ~((1 << LED_GREEN_PIN) | (1 << LED_RED_PIN));

    switch (state) {
        case 0: PORTD |= (1 << LED_BLUE_PIN); break;
        case 1: PORTB |= (1 << LED_GREEN_PIN); break;
        case 2: PORTB |= (1 << LED_RED_PIN); break;
    }
}

void beep_buzzer_short(void) {
    for (uint8_t i = 0; i < 2; i++) {
        PORTD |= (1 << BUZZER_PIN);
        _delay_ms(100);
        PORTD &= ~(1 << BUZZER_PIN);
        _delay_ms(100);
    }
}

void beep_buzzer_long(void) {
    PORTD |= (1 << BUZZER_PIN);
    _delay_ms(500);
    PORTD &= ~(1 << BUZZER_PIN);
}

void led_animation(void) {
    uint32_t start = systicks;
    while (systicks - start < LED_ANIMATION_TIME) {
        PORTD |= (1 << LED_BLUE_PIN); _delay_ms(150); PORTD &= ~(1 << LED_BLUE_PIN);
        PORTB |= (1 << LED_GREEN_PIN); _delay_ms(150); PORTB &= ~(1 << LED_GREEN_PIN);
        PORTB |= (1 << LED_RED_PIN); _delay_ms(150); PORTB &= ~(1 << LED_RED_PIN);
    }
    update_leds(current_state);
}

// ================== Timer Setup ==================

void Timer1_init_servos(void) {
    TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11);
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);
    ICR1 = 20000;
}

// ================== Control Servo ==================

void servo_set_angle(uint8_t id, uint8_t angle) {
    uint16_t pulse = (angle * 11) + 1000;
    if (pulse > 3200) pulse = 3200;

    switch (id) {
        case 0: OCR1A = pulse; break;
        case 1: OCR1B = pulse; break;
    }
}

void servo3_manual(uint8_t angle) {
    uint16_t pulse = (angle == 90) ? 1500 : 600;

    for (uint8_t i = 0; i < 25; i++) {
        PORTD |= (1 << SERVO3_PIN);
        if (pulse == 600) _delay_us(600);
        else _delay_us(1500);
        PORTD &= ~(1 << SERVO3_PIN);
        _delay_ms(18);
    }
}

void move_servo(uint8_t state) {
    beep_buzzer_short();
    led_animation();

    if (state == 0) {
        servo_set_angle(0, 180); _delay_ms(500);
        servo_set_angle(0, 0); _delay_ms(500);
    } else if (state == 1) {
        servo_set_angle(1, 180); _delay_ms(500);
        servo_set_angle(1, 0); _delay_ms(500);
    } else if (state == 2) {
        servo3_manual(90); _delay_ms(500);
        servo3_manual(0); _delay_ms(500);
    }
}

// ================== ISR ==================

ISR(INT0_vect) {
    static uint32_t last_interrupt = 0;
    if (systicks - last_interrupt > 300) {
        last_interrupt = systicks;
        right_button_flag = 1;
    }
}

// ================== Main ==================

int main(void) {
    GPIO_init();
    USART0_init(CALC_USART_UBRR(PM_BAUD));
    USART0_use_stdio();

    Timer0_init_ctc();
    Timer1_init_servos();
    interrupts_init();

    sei();

    update_leds(current_state);
    servo_set_angle(0, 0);
    servo_set_angle(1, 0);

    srand(ADC);

    printf("Sistem pornit.\n");

    uint8_t last_button_left = 1;
    uint8_t last_button_random = 1;
    uint32_t last_ping = 0;

    while (1) {
        if (SYSTICKS_PASSED(last_ping, 3000)) {
            last_ping = systicks;
            printf("Alive @ %lu ms\n", systicks);
        }

        uint8_t btn_left = (PIND & (1 << BUTTON_LEFT_PIN)) == 0;
        if (btn_left && last_button_left) {
            beep_buzzer_long();
            current_state = (current_state + 1) % 3;
            update_leds(current_state);
        }
        last_button_left = !btn_left;

        if (right_button_flag) {
            right_button_flag = 0;
            move_servo(current_state);
        }

        uint8_t btn_random = (PINB & (1 << BUTTON_RANDOM_PIN)) == 0;
        if (btn_random && last_button_random) {
            uint8_t random_servo = rand() % 3;
            move_servo(random_servo);
        }
        last_button_random = !btn_random;
    }
}
