/*
 *  bluetooth.c      HC-05 Bluetooth Driver
 *  UART-Bluetooth device driver
 *  Created by Andrew Stange
 */

// This USART1 is intended to be used with the HC-05 bluetooth module to allow for wireless communication. However, it
// can be used like a normal polling USART driver
// Breadboard + HC-05 Notes
//      HC-05 can take 3.3-6V from VCC, but needs 3.3V to its RX pins --> pass MCU TX through a resistance division circuit
//      All HC-05 pins besides STATE
//      Run KEY high to switch to AT mode (change module settings). Let KEY float, normal communication
//      Normal communication --> let KEY float and treat like USART
//      Default --> 9600 baud rate, 8 bit data, 1 stop bit, no parity, no flow control
//      AT mode --> 38400 baud
//      Default bluetooth password is 1234 (module name HC-05)



#include "kernel.h"
volatile int uart1_unavailable = 0;


// USART1_Init
void USART1_Init_Poll(void) {
    // GPIO and USART clock enable
    RCC->AHB1ENR |= 1;               // clock enable GPIOA
    RCC->APB2ENR |= (1 << 4);		// clock enable USART1

    // configure GPIO
    GPIOA->AFR[1] |= (7 << 4) | (7 << 8);       // USART1 AF7 for PA9 (TX) and PA10 (RX)
    GPIOA->MODER |= (2 << 18) | (2 << 20);      // enable alternate function for PA9, PA10

    // set up USART1
    USART1->BRR = 0x0683;			// set 9600 baud @ 16 Mhz
    USART1->CR1 |= (1 << 2) | (1 << 3) | (1 << 13);          // enable TX and RX
}   // END USART1_Init




// USART1_write
int USART1_write(int ch) {
    // wait for TX buffer to be empty before writing
    while(!(USART1->SR & 0x0080)) {
        threadYield();
    }

    // write ch to the data register
    USART1->DR = (ch & 0xFF);
    return ch;
}   // END USART1_write



// USART1_read
int USART1_read(void) {
    // wait for RX buffer to have something in it
    while(!(USART1->SR & 0x0020) ) {
        threadYield();
    }
    return USART1->DR;
}   // END USART1_read





// USART1_send_str
void USART1_send_str(char* buf) {
    __disable_irq();
    while(uart1_unavailable == 1) {
        __enable_irq();
        threadYield();
        __disable_irq();
    };
    uart1_unavailable = 1;
    __enable_irq();

    int i = 0;
    while(buf[i] != '\0') {
        USART1_write(buf[i++]);
    }
    uart1_unavailable = 0;
}





// USART1_send_integer
#define BASE_10  10
void USART1_send_int(short value) {
    // Convert the number to string
    char valueAsString[12];
    int_to_str(value, valueAsString, BASE_10);

    // Right justify display the string
    short strlen = 0;
    while(valueAsString[strlen]) {
        strlen++;
    }

    while(strlen < 7) {
        USART1_send_str(" ");
        ++strlen;
    }
    USART1_send_str(valueAsString);
}


