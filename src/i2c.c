/*
 *  i2c.c      I2C3 Driver
 *  I2C3 driver.  Originally intended for use with external altimeter module
 *  Created by Andrew Stange
 */

#include "kernel.h"




// I2C3 driver as well as middleware to read/write.  Originally intended for use with an external altimeter module
// Example init function included at bottom of file.  I2C parameters change from use case to use case


// I2C3_restart
void I2C3_restart (void) {
    // pg 492, STM32F411 ref manual
    I2C3->CR1 |= (1 << 8);
}   // END I2C3_restart



// I2C3_stop
void I2C3_stop (void) {
    // pg 492, STM32F411 ref manual
    I2C3->CR1 |= (1 << 9);
}   // END I2C3_stop


// I2C3_enable_ack
void I2C3_enable_ack (void) {
    // pg 492, STM32F411 ref manual
    // enable sending of an ACK once a byte is received (accel. will acknowledge when a byte received)
    I2C3->CR1 |= (1 << 10);
}   // END I2C3_enable_ack



// I2C3_disable_ack
void I2C3_disable_ack (void) {
    // pg 492, STM32F411 ref manual
    I2C3->CR1 &= ~(1 << 10);
}   // END I2C3_disable_ack



// I2C3_send_slave_addr
void I2C3_send_slave_addr (unsigned short addr) {
    // pg 497, STM32F411 ref manual
    // wait for a start condition
    while( (I2C3->SR1 & 1) == 0 );

    // load slave address into data register, to be transmitted
    I2C3->DR = addr;

    // pg 497, STM32F411 ref manual
    // wait for end of address transmission (from master POV)
    while( (I2C3->SR1 & (1 << 1)) == 0 );

    // pg 478, after reading address flag in SR1, read SR1 and SR2
    I2C3->SR1;
    I2C3->SR2;
}   // END I2C3_send_slave_addr



// I2C3_send_slave_addr
void I2C3_send_slave_addr_adjust (unsigned short addr) {
    // pg 497, STM32F411 ref manual
    // wait for a start condition
    while( (I2C3->SR1 & 1) == 0 );

    // load slave address into data register, to be transmitted
    I2C3->DR = addr;

    // pg 497, STM32F411 ref manual
    // wait for end of address transmission (from master POV)
    while( (I2C3->SR1 & (1 << 1)) == 0 );

    // pg 478, after reading address flag in SR1, read SR1 and SR2
    I2C3->SR1;
    I2C3->SR2;
}   // END I2C3_send_slave_addr_adjust




// I2C3_send_reg
void I2C3_send_reg (unsigned short addr) {
    // pg 497, STM32F411 ref manual
    // wait for end of transmission
    while( (I2C3->SR1 & (1 << 7)) == 0 );

    // load address into DR for transmission
    I2C3->DR = addr;

    // pg 497, STM32F411 ref manual
    // wait for end of transmission
    while( (I2C3->SR1 & (1 << 7)) == 0 );
}   // END I2C3_send_reg




// I2C3_busy_wait
void I2C3_busy_wait (void) {
    // pg 500, STM32F411 ref manual
    // wait while bus is busy
    while( (I2C3->SR2 & (1 << 1)) == (1 << 1) );
}   // END I2C3_busy_wait




// I2C3_write_byte
void I2C3_write_byte (unsigned char data) {
    // load data for transmission
    I2C3->DR = data;

    // pg 500, STM32F411 ref manual.  Wait until byte has been sent
    while( (I2C3->SR1 & (1 << 2)) == 0 );
}   // END I2C3_write_byte



// I2C3_get_data
unsigned char I2C3_get_data (void) {
    // pg 499, STM32F411 ref manual.  Wait while DR is empty
    while( (I2C3->SR1 & (1 << 6)) == 0 );

    // return DR contents
    return I2C3->DR;
}   // END I2C3_get_data










// alt_read
unsigned char alt_read (unsigned short addr) {
    // see pg 19, LSM303DLHC datasheet. Details interaction with sensor over I2C
    I2C3_busy_wait();
    I2C3_enable_ack();
    I2C3_restart();

    I2C3_send_slave_addr(ACCELEROMETER_WRITE);
    I2C3_send_reg(addr);
    I2C3_restart();

    I2C3_send_slave_addr(ACCELEROMETER_READ);
    I2C3_disable_ack();
    I2C3_stop();

    return I2C3_get_data();
}   // END alt_read







// accelerometer_write
void alt_write (unsigned short addr, unsigned char data) {
    // see pg 19, LSM303DLHC datasheet. Details interaction with sensor over I2C
    I2C3_busy_wait();
    I2C3_restart();

    I2C3_send_slave_addr(ACCELEROMETER_WRITE);
    I2C3_send_reg(addr);
    I2C3_write_byte(data);

    I2C3_stop();
}   // END accelerometer_write





// pg 33, STM32F411VE user manual
// SDA: PC9
// SCL: PA8
void alt_i2c_init (void) {
    /* Basic set up */
    // GPIOB clock enable
    RCC->AHB1ENR |= 1 | (1 << 2);

    // pg 157, STM32F411 ref manual
    // enable alternate function
    GPIOA->MODER |= (2 << 16); // PA8
    GPIOC->MODER |= (2 << 18); // PC9

    // I2C3 is AF4.  Pin 8 --> AFRH bits 0-3. Pin 9 --> AFRH bits 4-7
    GPIOA->AFR[1] |= 4;
    GPIOC->AFR[1] |= (4 << 4);

    // pins set to fast speed.  Pin 8 --> bit 16/17. Pin 9 --> 18/19
    GPIOA->OSPEEDR |= (2 << 16);
    GPIOC->OSPEEDR |= (2 << 18);

    // clock enable I2C3
    RCC->APB1ENR |= (1 << 23);


    /* Timing and Addressing configuration */
    // pg 495, STM32F411 ref manual
    // set peripheral clock frequency through CR2. Set to 2 MHz
    I2C3->CR2 &= ~(0x3F);
    I2C3->CR2 |= 0x02;


    // pg 502, STM32F411 ref manual
    // using Sm mode
    // Note example:
    //    "For instance: in Sm mode, to generate a 100 kHz SCL frequency:
    //     If FREQR = 08, TPCLK1 = 125 ns so CCR must be programmed with 0x28
    //     (0x28 <=> 40d x 125 ns = 5000 ns.)"
    // 125 ns = 8 MHz clock. Substitute in 62.5 for 125 and compensate by doubling
    // 40d to 80d which = 0x50.
    I2C3->CCR &= ~(0xFFF);
    I2C3->CCR |= 0x50;

    // pg 503, STM32F411 ref manual
    // Example, FREQ=0x08 and TPCLK1=125 ns --> 1000 ns / 125 ns = 8 + 1
    // (+1 according to TRISE register)
    // In our case, FREQ is set to 0x02 --> 1000 ns / 500 ns = 2 + 1
    I2C3->TRISE &= ~(0x3F);
    I2C3->TRISE |= 0x03;

    // pg 496, STM32F411 ref manual
    // Bit 1-7: interface address is 100001 = 0x21
    // Bit 14: always keep to 1
    // 7 bit addressing, bit 15 --> 0
    I2C3->OAR1 |= ((0x21 << 1) | (1 << 14));


    /* Enable I2C3 */
    I2C3->CR1 |= 1;
}   // END alt_i2c_init


