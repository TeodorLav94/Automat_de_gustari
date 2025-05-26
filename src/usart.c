#include <avr/io.h>
#include <stdio.h>
#include "usart.h"

static int uart_putchar(char c, FILE *stream);

// stream pentru printf
static FILE uart_output = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);

void USART0_init(uint16_t ubrr) {
    UBRR0H = (uint8_t)(ubrr >> 8);
    UBRR0L = (uint8_t)(ubrr);
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);   // Enable RX + TX
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // 8-bit data, no parity
}

void USART0_use_stdio(void) {
    stdout = &uart_output;
}

static int uart_putchar(char c, FILE *stream) {
    if (c == '\n') uart_putchar('\r', stream);  // CR înainte de LF pentru terminale
    while (!(UCSR0A & (1 << UDRE0)));           // Așteaptă buffer gol
    UDR0 = c;
    return 0;
}
