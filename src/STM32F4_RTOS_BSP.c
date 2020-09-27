/*
 *  STM32F4_RTOS_BSP.c      Board support package
 *  Implement constants and basic functions for STM32F4 boards
 *  Created by Andrew Stange
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
#define BSP_Button_PORT		GPIOA
#define BSP_Probe0_BIT      (1U<<0)   /*PC0 as output*/
#define BSP_Probe1_BIT      (1U<<2)   /*PC1 as output*/
#define BSP_Probe2_BIT      (1U<<4)   /*PC2 as output*/
#define BSP_Probe3_BIT      (1U<<8)   /*PC4 as output*/
#define BSP_Probe_PORT		GPIOC
#define CH0			        (1U<<0)
#define CH1			        (1U<<1)
#define CH2			        (1U<<2)
#define CH3			        (1U<<4)



/* Enable PC13 for edge-detectio n*/
void BSP_EdgeTrigger_Init(){
   __disable_irq();
	RCC->AHB1ENR |=4;
	RCC->APB2ENR |=0x4000;
	GPIOC->MODER &= ~0x0C000000;        /*Clear pin mode to input mode*/
	SYSCFG->EXTICR[3] &= ~0x00F0;       /*Clear port selection for EXTI13*/
	SYSCFG->EXTICR[3] |= 0x0020;		/*Select port C for EXTI13*/
	EXTI->IMR |= 0x2000;                /*unmask EXTI13*/
	EXTI->FTSR |=0x2000;	            /*select falling edge trigger*/
	NVIC_EnableIRQ(EXTI15_10_IRQn);
	__enable_irq();
}




void BSP_Probe_Init(void){
    RCC->AHB1ENR |= GPIOC_CLOCK;
    BSP_Probe_PORT->MODER |=  BSP_Probe0_BIT|BSP_Probe1_BIT|BSP_Probe2_BIT|BSP_Probe3_BIT;
}


void BSP_Probe_CH0(){
    BSP_Probe_PORT->ODR ^= CH0;
}


void BSP_Probe_CH1(){
    BSP_Probe_PORT->ODR ^= CH1;
}


void BSP_Probe_CH2(){
    BSP_Probe_PORT->ODR ^= CH2;
}


void BSP_Probe_CH3(){
    BSP_Probe_PORT->ODR ^= CH3;
}


void BSP_Button_Init(void)
{
    RCC->AHB1ENR |= GPIOA_CLOCK;
    BSP_Button_PORT->MODER &= ~0x00000011; /*Clear PA0 pin*/
}


uint32_t  BSP_Button_Read(void){
    return BSP_Button_PORT->IDR &0x01;
}


void BSP_Delay_Millisecond(uint32_t delay){
    RCC->APB1ENR |= 0x02;  /*Enable TIM3 clock*/
	TIM3->PSC = 160 -1;  /* 16 000 000 /160 = 100 000*/
	TIM3->ARR = 100 -1;  /* 100 000 /100 = 1000*/
	TIM3->CNT =0;
	TIM3->CR1 =1;
	for(int i =0;i<delay;i++){
	    while(!(TIM3->SR & 1)){}   /*wait for UIF set*/
	    TIM3->SR &= ~1;
	}
}


void  BSP_ADC1_Init(void){
	/*GPIO Pin*/
	RCC->AHB1ENR |=1;				/*ENABLE clock for GPIOA*/
	GPIOA->MODER |= 0xC;		/*PA1 set as analog*/
	
	/*Setup ADC!*/
	RCC->APB2ENR |=0x00000100;
	ADC1->CR2		=0;
	ADC1->SQR3  =1;  /*Conversion sequence start at channel 1*/
	ADC1->SQR1  =0;  /*Length of sequence is 1*/
	ADC1->CR2		|=1;
}


uint32_t BSP_Sensor_Read(void) {
    ADC1->CR2 |= 0x40000000;   /*Start conversion*/
    while(!(ADC1->SR & 2)){}   /*Wait for conversion complete*/
    return ADC1->DR;
}


void BSP_LED_Init(void) {
  __disable_irq();
  RCC->AHB1ENR |=GPIOD_CLOCK;
  GPIOD->MODER |= BSP_LED_red_BIT|BSP_LED_green_BIT|BSP_LED_orange_BIT|BSP_LED_blue_BIT;
  __enable_irq();
}


void BSP_LED_blueOn(void) {
    GPIOD->ODR |=BSP_LED_blue;
}

void BSP_LED_blueOff(void) {
    GPIOD->ODR &= ~BSP_LED_blue;
}

void BSP_LED_orangeOn(void) {
    GPIOD->ODR |=BSP_LED_orange;
}

void BSP_LED_orangeOff(void){
    GPIOD->ODR &= ~BSP_LED_orange;
}

void BSP_LED_greenOn(void) {
    GPIOD->ODR |=BSP_LED_green;
}

void BSP_LED_greenOff(void) {
    GPIOD->ODR &= ~BSP_LED_green;
}

void BSP_LED_redOn(void) {
    GPIOD->ODR |=BSP_LED_red;
}

void BSP_LED_redOff(void) {
    GPIOD->ODR &= ~BSP_LED_red;
}

void BSP_LED_blueToggle(void) {
    GPIOD->ODR ^=BSP_LED_blue;
}

void BSP_LED_greenToggle(void) {
    GPIOD->ODR ^=BSP_LED_green;
}

void BSP_LED_redToggle(void) {
    GPIOD->ODR ^=BSP_LED_red;
}

void BSP_LED_orangeToggle(void) {
    GPIOD->ODR ^=BSP_LED_orange;
}

