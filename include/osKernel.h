/*
 *  osKernel.h      Kernel header file
 *  Provides interface with kernel functions for main.c
 *  Created by Andrew Stange
 */

#ifndef __OS_KERNEL_H__
#define __OS_KERNEL_H__

// header files
#include <stdint.h>
// #include <stdio.h>                      // allow for simplified IO over USART (fgetc/fputc overriden in uart.c)
#include "stm32f4xx.h"                  // Device header
#include "STM32F4_RTOS_BSP.h"           // Board support package

#define	QUANTA	                30
#define MAX_UART_DMA_BUFFER     2000
#define DMA1_6_PRI              10
#define USART2_PRI              10
#define PERIODIC_PRI            8       // needs to be the same as PendSV so periodic task cannot be preempted
                                        // PendSV should wait until no interrupts are active before executing, but unsure
                                        // about its behavior if interrupt is pending --> need same priority to avoid
                                        // PendSV performing a context switch while (or before) servicing the periodic task

// Basic Kernel
void osKernelLaunch(void);
void osKernelInit(void);
int8_t osKernelAddThreads( void(*task)(void), uint32_t priority );
uint8_t osKernelAddPeriod_Thread(void (*task)(void), uint32_t period);
void osThreadYield(void);
void osThreadSleep(uint32_t sleep_time);
void busy_wait(int t);
void osReturnHandler(void);

// UART (other UART functions are intermediaries for STDIO function)
void USART2_Init_Poll(void);
void USART2_Init(void);
int USART2_write(int ch);
int USART2_read(void);
void USART2_send_int(short value);
void USART2_send_str(char* str);
void USART1_Init(void);
int USART1_write(int ch);
int USART1_read(void);

// AHRS
void gyroInit(void);
void gyroGetValues(int16_t* x, int16_t* y, int16_t* z);
short getAxisValue(unsigned char lowRegister, unsigned char highRegister);
void accelerometer_init (void);
void accelGetValues(int16_t* x, int16_t* y, int16_t* z);

// String Utils
char* int_to_str(int value, char *result, int base);


// Semaphores
void osSemaphoreInit(int32_t *semaphore,int32_t value);
void osSignalSet(int *semaphore);
void osSignalWait(int32_t *semaphore);


// Mailbox
void osMailboxInit(void);
void osMailboxSend(uint32_t data);
uint32_t osMailboxRecv(void);


// FIFO
void osFifoInit(void);
int8_t osFifoPut(uint32_t data);
uint32_t osFifoGet(void);


#endif























