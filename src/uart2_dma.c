/*
 *  uart_dma.c      UART-DMA function implementation
 *  UART DMA driver
 *  Created by Andrew Stange
 */


//  DMA Notes (from Memory-->Peripheral perspective)
//      SxPAR register holds peripheral data register
//      SxMOAR holds memory address of data to transfer
//      SxNDTR holds number of data items to transfer
//      SxCR to specify M-P communication, mem pointer increment, channel selection, and stream priority


#include "kernel.h"


volatile int uart_unavailable = 0;
unsigned char uart_dma_buffer[MAX_UART_DMA_BUFFER];


// DMA1_Stream6_IRQHandler   (ref manual pg 188)
void DMA1_Stream6_IRQHandler(void) {
    // check stream transfer complete interrupt flag (TCIF)
    // TCIF is bit 21.  Set by hardware, clear by writing 1 to HIFCR
    if(DMA1->HISR & (1 << 21)) {
        DMA1->HIFCR |= (1 << 21);    // clear interrupt flag

        // pg 551, set USART to generate interrupt when transmission complete
        USART2->CR1 |= (1 << 6);
    }
    asm("dsb");
    asm("isb");                     // flush pipeline to ensure IRQ actions take effect
}   // END DMA1_Stream6_IRQHandler




// USART2_IRQHandler
void USART2_IRQHandler(void) {
    // pg 549 ref manual
    // bit 6 set when transmission complete
    if(USART2->SR & (1 << 6)) {
        // clear interrupt flag
        USART2->SR &= ~(1 << 6);

        // clear flag to allow next transmission
        uart_unavailable = 0;
    }
    asm("dsb");
    asm("isb");                     // flush pipeline to ensure IRQ actions take effect
}   // END USART2_IRQHandler




// USART2_Init
void USART2_Init(void) {
    /*
     *  GPIO set up
     */
    // GPIOA clock access
    RCC->AHB1ENR |= 1;

    // pg 19 user manual --> PA2 USART2_TX, PA3 USART2_RX
    // pg 157 ref manual --> set PA2 and PA3 to alternate function
    //GPIOA->MODER |= (1 << 5) | (1 << 7);
    GPIOA->MODER |= 0x00A0;			// enable alternate function for PA2, PA3

    // page 150, ref manual --> USART2 is AF7 (set for both PA2 and PA3)
    //GPIOA->AFR[0] |= (7 << 8) | (7 << 12);
    GPIOA->AFR[0] |= 0x7700;		// Alt 7 (for both RX and TX)

    // set GPIO speed to low (pg 158, ref manual) --> 00 for low speed, clear bits
 //   GPIOA->OSPEEDR &= ~((3 << 4) | (3 << 6));
  //  GPIOA->OSPEEDR |= ((3 << 4) | (3 << 6));

    /*
     *  DMA set up
     */
    // DMA1 clock enable (pg 117, ref manual)
    RCC->AHB1ENR |= (1 << 21);

    // pg 190, ref manual
    // set bit 6 for memory to peripheral communication
    // set bit 10 to increment memory pointer after each memory transfer
    // set bit 27 to use channel 4, stream 6 (pg 170, ref manual)
    DMA1_Stream6->CR |= (1 << 6) | (1 << 10) | (1 << 27);

    // enable DMA1_Stream6 interrupt (pg 203, ref manual)
    __disable_irq();
    NVIC_SetPriority(DMA1_Stream6_IRQn, DMA1_6_PRI);
    NVIC_EnableIRQ(DMA1_Stream6_IRQn);
    __enable_irq();


    /*
     *  USART Init
     */
    // clock enable USART2 (pg 118, ref manual)
    RCC->APB1ENR |= (1 << 17);

    // pg 522 and 550, ref manual for baud rate calculation specifics
    // peripheral clock = 16MHz
    // 0x683 --> 0110 1000 0011
    // fraction = 0011, mantissa = 1101 0000 --> USARTDIV = 208.3
    // baud rate of 9600
    USART2->BRR = 0x0683;

    // Enable DMA transmission for USART2 (pg 555, ref manual)
    USART2->CR3 |= (1 << 7);

    // enable reception, transmission, and USART2 itself
    USART2->CR1 |= (1 << 2) | (1 << 3) | (1 << 13);

    // enable USART2 interrupt
    // (USART2 interrupt enabled inside the DMA IRQhandler since DMA interrupt
    // will precede corresponding USART interrupt)
    __disable_irq();
    NVIC_SetPriority(USART2_IRQn, USART2_PRI);
    NVIC_EnableIRQ(USART2_IRQn);
    __enable_irq();
}   // END USART2_init





// USART2_send_str
void USART2_send_str(char* str) {
    // use uart_unavailable as a semaphore, avoid multiple USART transmissions
    // clobbering each other
    __disable_irq();
    while(uart_unavailable == 1) {
        __enable_irq();
        threadYield();
        __disable_irq();
    };
    uart_unavailable = 1;
    __enable_irq();

    // copy string into DMA buffer (until max size or NULL byte)
    int str_len = 0;
    while(str_len < MAX_UART_DMA_BUFFER && str[str_len]) {
        uart_dma_buffer[str_len] = str[str_len];
        ++str_len;
    }

    // pg 193, ref manual. Contain number of data items to transfer
    // 0-65535 (16 bit register).  Read only when stream enabled
    DMA1_Stream6->NDTR = str_len;

    // pg 194, ref manual
    // address of data register of peripheral destination for the transfer
    DMA1_Stream6->PAR = 0x40004404;     // needs to be a raw address, cannot be CMSIS struct

    // pg 194, ref manual
    // address of data to transfer to the peripheral
    DMA1_Stream6->M0AR = (unsigned int) uart_dma_buffer;

    // Enable interrupt on complete transmission (pg 192, ref manual)
    DMA1_Stream6->CR |= (1 << 4);

    // Enable DMA stream (pg 190, ref manaul)
    DMA1_Stream6->CR |= 1;
}   // USART2_send_str




// USART2_send_integer
#define BASE_10  10
void USART2_send_int(short value) {
    // Convert the number to string
    char valueAsString[12];
    int_to_str(value, valueAsString, BASE_10);

    // Right justify display the string
    short strlen = 0;
    while(valueAsString[strlen]) {
        strlen++;
    }

    while(strlen < 7) {
        USART2_send_str(" ");
        ++strlen;
    }
    USART2_send_str(valueAsString);
}   // END USART2_send_integer





