/*
 *  usart1.c      USART1
 *  USART1 driver (DMA RX) --> Written for Bluetooth module RX
 */

#include "kernel.h"

// PB6  TX
// PB7  RX


// USART1_Init
void USART1_Init(void) {
    // GPIO and USART clock enable
    RCC->AHB1ENR |= 2;              // clock enable GPIOB
    RCC->APB2ENR |= (1 << 4);		// clock enable USART1

    // set alternate function
    GPIOB->AFR[0] |= ((7 << 24) | (7 << 28));   // USART1 is AF7 (7 for TX (PB6) and 7 for RX (PB7))
    GPIOB->MODER |= ((2 << 12) | (2 << 14));	// enable alternate function for PB6 and PB7

    // set up USART1
    USART1->BRR = 0x0683;			// set 9600 baud @ 16 Mhz
    USART1->CR1 |= 0x200C;          // enable TX and RX
}   // END USART1_Init