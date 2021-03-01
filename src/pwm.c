/*
 *  pwm.c      PWM with Timers
 *  PWM with TIM.  Originally used for controlling electric motors
 *  Created by Andrew Stange
 */
#include "osKernel.h"


// pwm_init
void pwm_init(void) {
    // clock enable
    RCC->AHB1ENR |= 0x07;           // GPIOA,B,C clock access  TODO enable additional GPIO if needed
    RCC->APB1ENR |= 0x03;           // TIM2,3 clock enable
    RCC->APB2ENR |= 0x600;          // TIM10,11 clock enable

    // TIM2: PA0
    // TIM3: PC6
    // TIM9: PA2
    // TIM10: PB8
    // GPIO
    GPIOA->AFR[0] |= 1;             // set PA0 alt. function --> TIM2 channel 1 (AF1)
    GPIOA->MODER |= 2;              // alternate function for PA0

    GPIOB->AFR[1] |= 3;             // set PB8 alt. function --> TIM10 channel 1 (AF3)
    GPIOB->MODER |= (2 << 16);      // alternate function for PB8

    GPIOC->AFR[0] |= (2 << 24);     // set PC6 alt. function --> TIM3 channel 1 (AF2)
    GPIOC->MODER |= (2 << 12);      // alternate function for PC6

    // TIM2
    TIM2->PSC = PWM_PSC - 1;        // prescalar of 10
    TIM2->ARR = (BUS_FREQ / (PWM_PSC * PWM_FREQ)) - 1;
    TIM2->CNT = 0;                  // clear count register
    TIM2->CCMR1 = 0x60;             // enable PWM mode
    TIM2->CCER = 1;                 // enable PWM ch1
    TIM2->CCR1 = 0;                 // start at 0
    TIM2->CR1 |= 1;                 // start clock

    // TIM3
    TIM3->PSC = PWM_PSC - 1;        // prescalar of 10
    TIM3->ARR = (BUS_FREQ / (PWM_PSC * PWM_FREQ)) - 1;
    TIM3->CNT = 0;                  // clear count register
    TIM3->CCMR1 = 0x60;             // enable PWM mode
    TIM3->CCER = 1;                 // enable PWM ch1
    TIM3->CCR1 = 0;                 // start at 0
    TIM3->CR1 |= 1;                 // start clock

    // TIM10
    TIM10->PSC = PWM_PSC - 1;       // prescalar of 10
    TIM10->ARR = (BUS_FREQ / (PWM_PSC * PWM_FREQ)) - 1;
    TIM10->CNT = 0;                 // clear count register
    TIM10->CCMR1 = 0x60;            // enable PWM mode
    TIM10->CCER = 1;                // enable PWM ch1
    TIM10->CCR1 = 0;                // start at 0
    TIM10->CR1 |= 1;                // start clock

    // TIM9     Shares a pin with USART2, keep commented to allow for use of USART
//    TIM9->PSC = PWM_PSC - 1;        // prescalar of 10
//    TIM9->ARR = (BUS_FREQ / (PWM_PSC * PWM_FREQ)) - 1;
//    TIM9->CNT = 0;                  // clear count register
//    TIM9->CCMR1 = 0x60;             // enable PWM mode
//    TIM9->CCER = 1;                 // enable PWM ch1
//    TIM9->CCR1 = 0;                 // start at 0
//    TIM9->CR1 |= 1;                 // start clock
}   // END pwm_init




