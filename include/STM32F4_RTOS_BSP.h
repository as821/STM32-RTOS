/*
 *  STM32F4_RTOS_BSP.h      Board support package header
 *  Expose constants and basic functions for STM32F4 boards
 *  Created by Andrew Stange
 */


#ifndef __STM32F4_RTOS_BSP_H
#define __STM32F4_RTOS_BSP_H
#include <stdint.h>

// LED
void 	BSP_LED_Init(void);
void BSP_LED_blueOn(void);
void BSP_LED_blueOff(void);
void BSP_LED_orangeOn(void);
void BSP_LED_orangeOff(void);
void BSP_LED_redOn(void);
void BSP_LED_redOff(void);
void BSP_LED_greenOn(void);
void BSP_LED_greenOff(void);
void BSP_LED_orangeToggle(void);
void BSP_LED_blueToggle(void);
void BSP_LED_greenToggle(void);
void BSP_LED_redToggle(void);

// GPIO
uint32_t BSP_Sensor_Read(void);
void BSP_Probe_Init(void);
void BSP_Probe_CH0(void);
void BSP_Probe_CH1(void);
void BSP_Probe_CH2(void);
void BSP_Probe_CH3(void);

// TIM
void BSP_TIM2_Init(void);
void BSP_Delay_Millisecond(uint32_t delay);

// MISC
void BSP_EdgeTrigger_Init(void);
void  BSP_ADC1_Init(void);
void BSP_Button_Init(void);
uint32_t  BSP_Button_Read(void);

#endif
