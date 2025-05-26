#ifndef USART_H
#define USART_H

#include <stdint.h>

#define CALC_USART_UBRR(baud) ((F_CPU / (16UL * (baud))) - 1)

void USART0_init(uint16_t ubrr);
void USART0_use_stdio(void);

#endif
