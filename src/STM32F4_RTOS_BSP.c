/*
 *  STM32F4_RTOS_BSP.c      Board support package
 *  Implement constants and basic functions for STM32F4 boards
 */

#include "STM32F4_RTOS_BSP.h"
#include "stm32f4xx.h"                  // Device header

#define BSP_LED_red_BIT     (1U<<28)
#define BSP_LED_green_BIT   (1U<<24)
#define BSP_LED_orange_BIT  (1U<<26)
#define BSP_LED_blue_BIT    (1U<<30)
#define	LED_PORT		    GPIOD
#define	BSP_LED_red		    (1U<<14)
#define BSP_LED_green       (1U<<12)
#define BSP_LED_blue        (1U<<15)
#define BSP_LED_orange      (1U<<13)
#define GPIOD_CLOCK         (1<<3)
#define GPIOA_CLOCK 	    (1<<0)
#define GPIOC_CLOCK         (1<<2)


void BSP_TIM2_Init(void){
    RCC->APB1ENR |=1;
    TIM2->PSC = 16000-1;    // divide system clock by 16000
    TIM2->ARR = 1000-1;     // reset value = 1000
    TIM2->CR1 = 1;
    TIM2->DIER |= 1;
    NVIC_EnableIRQ(TIM2_IRQn);
}

void BSP_Delay_Millisecond(uint32_t delay){
    RCC->APB1ENR |= 0x02;
	TIM3->PSC = 160 - 1;     // 16,000,000 /160 = 100 000
	TIM3->ARR = 100 - 1;     // 100,000 /100 = 1000
	TIM3->CNT =0;
	TIM3->CR1 =1;
	for(int i =0;i<delay;i++){
	    while(!(TIM3->SR & 1)){}
	    TIM3->SR &= ~1;
	}
}

void  BSP_ADC1_Init(void){
	RCC->AHB1ENR |=1;				/*ENABLE clock for GPIOA*/
	GPIOA->MODER |= 0xC;		/*PA1 set as analog*/
	
	RCC->APB2ENR |=0x00000100;
	ADC1->CR2 = 0;
	ADC1->SQR3 = 1;  /*Conversion sequence start at channel 1*/
	ADC1->SQR1 = 0;  /*Length of sequence is 1*/
	ADC1->CR2 |= 1;
}

uint32_t BSP_Sensor_Read(void) {
    ADC1->CR2 |= 0x40000000;   /*Start conversion*/
    while(!(ADC1->SR & 2)){}   /*Wait for conversion complete*/
    return ADC1->DR;
}

void BSP_LED_Init(void) {
    RCC->AHB1ENR |= GPIOD_CLOCK;
    GPIOD->MODER |= BSP_LED_red_BIT|BSP_LED_green_BIT|BSP_LED_orange_BIT|BSP_LED_blue_BIT;
}

void BSP_LED_blueOn(void) {
    GPIOD->ODR |= BSP_LED_blue;
}

void BSP_LED_blueOff(void) {
    GPIOD->ODR &= ~BSP_LED_blue;
}

void BSP_LED_orangeOn(void) {
    GPIOD->ODR |= BSP_LED_orange;
}

void BSP_LED_orangeOff(void){
    GPIOD->ODR &= ~BSP_LED_orange;
}

void BSP_LED_greenOn(void) {
    GPIOD->ODR |= BSP_LED_green;
}

void BSP_LED_greenOff(void) {
    GPIOD->ODR &= ~BSP_LED_green;
}

void BSP_LED_redOn(void) {
    GPIOD->ODR |= BSP_LED_red;
}

void BSP_LED_redOff(void) {
    GPIOD->ODR &= ~BSP_LED_red;
}

void BSP_LED_blueToggle(void) {
    GPIOD->ODR ^= BSP_LED_blue;
}

void BSP_LED_greenToggle(void) {
    GPIOD->ODR ^= BSP_LED_green;
}

void BSP_LED_redToggle(void) {
    GPIOD->ODR ^= BSP_LED_red;
}

void BSP_LED_orangeToggle(void) {
    GPIOD->ODR ^= BSP_LED_orange;
}

