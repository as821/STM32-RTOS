/*
 *  uart.c      UART function implementation
 *  UART driver and STDIO overriding (commented out at bottom of file)
 *  Created by Andrew Stange
 */


#include "kernel.h"

// USART2_Init
void USART2_Init_Poll(void) {
    // GPIO and USART clock enable
    RCC->AHB1ENR |= 1;              // clock enable GPIOA
    RCC->APB1ENR |= 0x20000;		// clock enable USART2

    // set alternate function
    GPIOA->AFR[0] |= 0x7700;		// Alt 7 (for both RX and TX)
    GPIOA->MODER |= 0x00A0;			// enable alternate function for PA2, PA3

    // set up USART2
    USART2->BRR = 0x0683;			// set 9600 baud @ 16 Mhz
    USART2->CR1 |= 0x200C;          // enable TX and RX
}   // END USART2_Init




// USART2_write
int USART2_write(int ch) {
    // wait for TX buffer to be empty before writing
    while(!(USART2->SR & 0x0080)) {
        threadYield();
    }

    // write ch to the data register
    USART2->DR = (ch & 0xFF);
    return ch;
}   // END USART2_write




// USART2_read
int USART2_read(void) {
    // wait for RX buffer to have something in it
    while(!(USART2->SR & 0x0020) ) {
        threadYield();
    }
    return USART2->DR;
}   // END USART2_read







//// connect embedded interface to STDIO library.  Can now use printf, etc.
//// set up for IO with C stdlib
//struct __FILE{int handle;};
//FILE __stdin = {0};
//FILE __stdout = {1};
//FILE __stderr = {2};
//
//// fgetc
//int fgetc(FILE* f) {
//    return USART1_read();
//}   // END fgetc
//
//
//
//// fputc
//int fputc(int ch, FILE* f) {
//    return USART1_write(ch);
//}   // END fputc




