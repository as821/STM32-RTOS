/*
 *  kernel.h      Kernel header file
 *  Provides interface with kernel functions for main.c
 *  Created by Andrew Stange
 */

#ifndef __KERNEL_H__
#define __KERNEL_H__

// header files
#include <stdint.h>
#include <math.h>
#include "stm32f4xx.h"                  // Device header
#include "STM32F4_RTOS_BSP.h"           // Board support package

// system constants
#define MAX_UART_DMA_BUFFER     2000
#define BUS_FREQ		        16000000
#define PI                      3.14159265
#define TIM5_PRI                6       // priority for sensor measurement
#define DMA1_6_PRI              10
#define USART2_PRI              10
#define PERIODIC_PRI            8       // needs to be the same as PendSV so periodic task cannot be preempted

#define ACCELEROMETER_READ  0x33
#define ACCELEROMETER_WRITE 0x32

// system settings
#define NUM_OF_THREADS          3
#define NUM_PERIODIC_TASKS      5
#define STACKSIZE               8500
#define PERIOD_STACK_SIZE       500
#define	QUANTA	                15
#define MAG_ORIENT              1000
#define FIFO_SIZE               5

#define PWM_FREQ               100             // number of times per second TIM2 will trigger
#define PWM_PSC                10



// Basic Kernel
void kernelLaunch(void);
void kernelInit(void);
int8_t kernelAddThreads( void(*task)(void), uint32_t priority );
uint8_t kernelAddPeriod_Thread(void (*task)(void), uint32_t period);
void threadYield(void);
void threadSleep(uint32_t sleep_time);
void busy_wait(float t);
void returnHandler(void);


// UART (other UART functions are intermediaries for STDIO function)
void USART2_Init_Poll(void);
void USART2_Init(void);
int USART2_write(int ch);
int USART2_read(void);
void USART2_send_int(short value);
void USART2_send_str(char* str);
void USART1_Init_Poll(void);
int USART1_write(int ch);
int USART1_read(void);
void USART1_send_int(short value);
void USART1_send_str(char* buf);


// Gyro
void gyroInit(void);
void gyroGetValues(short* x, short* y, short* z);
short getAxisValue(unsigned char lowRegister, unsigned char highRegister);

// Accel
void accelerometer_init (void);
void accelGetValues(short* x, short* y, short* z);

// Magnetometer
void mag_orientation(void);
void mag_init(void);
uint8_t magGetValues(int16_t* x, int16_t* y, int16_t* z);
double mag_heading_calc(int16_t x_val, int16_t y_val, int16_t z_val);
double mag_angle_calc(double cur_heading);
void magnetometer_write(unsigned short addr, unsigned char data);

// PWM
void pwm_init(void);

// Formatted Output (from external sources)
void DisplayHeader(void);
void DisplayIntegerValue(short value);
void DisplayAxisValue(char* label, short accel, short gyro);
void DisplayAxisValues(short accelX, short accelY, short accelZ, short gyroX, short gyroY, short gyroZ);
void DisplayIMUValue(char* label, short val);
void DisplayIMUValues(short roll, short pitch, short yaw);

// String Utils
char* int_to_str(int value, char *result, int base);

// Util
double buf_avg(int16_t* buf, uint16_t len);
double to_degree(double val);
void iir_lpf(int16_t* buf, uint16_t len, float beta, double* smooth);
double map(double x, int16_t in_min, int16_t in_max, int16_t out_min, int16_t out_max);

// I2C
unsigned char I2C1_get_data (void);
void I2C1_write_byte (unsigned char data);
void I2C1_busy_wait (void);
void I2C1_send_reg (unsigned short addr);
void I2C1_send_slave_addr (unsigned short addr);
void I2C1_disable_ack (void);
void I2C1_enable_ack (void);
void I2C1_stop (void);
void I2C1_restart (void);

// SPI
void SPI1_RX_wait(void);
void SPI1_TX_wait(void);

// Semaphores
void semaphore_init(int32_t *semaphore,int32_t value);
void semaphore_set(int32_t *semaphore);
void semaphore_wait(int32_t *semaphore);

// FIFO
void fifoInit(void);
int8_t fifoPut(uint32_t data);
uint32_t fifoGet(void);


#endif























