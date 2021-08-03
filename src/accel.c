/*
 *  accel.c      Accelerometer Driver
 *  LSM303DLHC Accelerometer driver over I2C
 *  Created by Andrew Stange
 */


// see pg 21 LSM303DLHC datasheet.  R/W addresses for accelerometer
#define ACCELEROMETER_READ  0x33
#define ACCELEROMETER_WRITE 0x32

#include "kernel.h"


// I2C1_restart
void I2C1_restart (void) {
    // pg 492, STM32F411 ref manual
    I2C1->CR1 |= (1 << 8);
}   // END I2C1_restart



// I2C1_stop
void I2C1_stop (void) {
    // pg 492, STM32F411 ref manual
    I2C1->CR1 |= (1 << 9);
}   // END I2C1_stop


// I2C1_enable_ack
void I2C1_enable_ack (void) {
    // pg 492, STM32F411 ref manual
    // enable sending of an ACK once a byte is received (accel. will acknowledge when a byte received)
    I2C1->CR1 |= (1 << 10);
}   // END I2C1_enable_ack



// I2C1_disable_ack
void I2C1_disable_ack (void) {
    // pg 492, STM32F411 ref manual
    I2C1->CR1 &= ~(1 << 10);
}   // END I2C1_disable_ack



// I2C1_send_slave_addr
void I2C1_send_slave_addr (unsigned short addr) {
    // pg 497, STM32F411 ref manual
    // wait for a start condition
    while( (I2C1->SR1 & 1) == 0 );

    // load slave address into data register, to be transmitted
    I2C1->DR = addr;

    // pg 497, STM32F411 ref manual
    // wait for end of address transmission (from master POV)
    while( (I2C1->SR1 & (1 << 1)) == 0 );

    // pg 478, after reading address flag in SR1, read SR1 and SR2
    I2C1->SR1;
    I2C1->SR2;
}   // END I2C1_send_slave_addr



// I2C1_send_slave_addr
void I2C1_send_slave_addr_adjust (unsigned short addr) {
    // pg 497, STM32F411 ref manual
    // wait for a start condition
    while( (I2C1->SR1 & 1) == 0 );

    // load slave address into data register, to be transmitted
    I2C1->DR = addr;

    // pg 497, STM32F411 ref manual
    // wait for end of address transmission (from master POV)
    while( (I2C1->SR1 & (1 << 1)) == 0 );

    // pg 478, after reading address flag in SR1, read SR1 and SR2
    I2C1->SR1;
    I2C1->SR2;
}   // END I2C1_send_slave_addr_adjust




// I2C1_send_reg
void I2C1_send_reg (unsigned short addr) {
    // pg 497, STM32F411 ref manual
    // wait for end of transmission
    while( (I2C1->SR1 & (1 << 7)) == 0 );

    // load address into DR for transmission
    I2C1->DR = addr;

    // pg 497, STM32F411 ref manual
    // wait for end of transmission
    while( (I2C1->SR1 & (1 << 7)) == 0 );
}   // END I2C1_send_reg




// I2C1_busy_wait
void I2C1_busy_wait (void) {
    // pg 500, STM32F411 ref manual
    // wait while bus is busy
    while( (I2C1->SR2 & (1 << 1)) == (1 << 1) );
}   // END I2C1_busy_wait




// I2C1_write_byte
void I2C1_write_byte (unsigned char data) {
    // load data for transmission
    I2C1->DR = data;

    // pg 500, STM32F411 ref manual.  Wait until byte has been sent
    while( (I2C1->SR1 & (1 << 2)) == 0 );
}   // END I2C1_write_byte



// I2C1_get_data
unsigned char I2C1_get_data (void) {
    // pg 499, STM32F411 ref manual.  Wait while DR is empty
    while( (I2C1->SR1 & (1 << 6)) == 0 );

    // return DR contents
    return I2C1->DR;
}   // END I2C1_get_data




// accelerometer_read
unsigned char accelerometer_read (unsigned short addr) {
    // see pg 19, LSM303DLHC datasheet. Details interaction with sensor over I2C
    I2C1_busy_wait();
    I2C1_enable_ack();
    I2C1_restart();

    I2C1_send_slave_addr(ACCELEROMETER_WRITE);
    I2C1_send_reg(addr);
    I2C1_restart();

    I2C1_send_slave_addr(ACCELEROMETER_READ);
    I2C1_disable_ack();
    I2C1_stop();

    return I2C1_get_data();
}   // END accelerometer_read







// accelerometer_write
void accelerometer_write (unsigned short addr, unsigned char data) {
    // see pg 19, LSM303DLHC datasheet. Details interaction with sensor over I2C
    I2C1_busy_wait();
    I2C1_restart();

    I2C1_send_slave_addr(ACCELEROMETER_WRITE);
    I2C1_send_reg(addr);
    I2C1_write_byte(data);

    I2C1_stop();
}   // END accelerometer_write





// accelerometer_init
// pg 33, STM32F411VE user manual
// SCL --> PB6
// SDA --> PB9
void accelerometer_init (void) {
    // GPIOB clock enable
    RCC->AHB1ENR |= (1 << 1);

    // pg 157, STM32F411 ref manual
    // enable alternate function for PB6 and PB9
    GPIOB->MODER |= ((1 << 13) | (1 << 19));

    // I2C1 is AF4.  Pin 6 --> AFRL bits 24-27. Pin 9 --> AFRH bits 4-7
    GPIOB->AFR[0] |= (4 << 24);
    GPIOB->AFR[1] |= (4 << 4);

    // pins set to fast speed.  Pin 6 --> bit 12/13. Pin 9 --> 18/19
    GPIOB->OSPEEDR |= ((2 << 12) | (2 << 18));

    // clock enable I2C1
    RCC->APB1ENR |= (1 << 21);

    // pg 495, STM32F411 ref manual
    // set peripheral clock frequency through CR2. Set to 2 MHz
    I2C1->CR2 &= ~(0x3F);
    I2C1->CR2 |= 0x02;


    // pg 502, STM32F411 ref manual
    // using Sm mode
    // Note example:
    //    "For instance: in Sm mode, to generate a 100 kHz SCL frequency:
    //     If FREQR = 08, TPCLK1 = 125 ns so CCR must be programmed with 0x28
    //     (0x28 <=> 40d x 125 ns = 5000 ns.)"
    // 125 ns = 8 MHz clock. Substitute in 62.5 for 125 andcompensate by doubling
    // 40d to 80d which = 0x50.
    I2C1->CCR &= ~(0xFFF);
    I2C1->CCR |= 0x50;

    // pg 503, STM32F411 ref manual
    // Example, FREQ=0x08 and TPCLK1=125 ns --> 1000 ns / 125 ns = 8 + 1
    // (+1 according to TRISE register)
    // In our case, FREQ is set to 0x02 --> 1000 ns / 500 ns = 2 + 1
    I2C1->TRISE &= ~(0x3F);
    I2C1->TRISE |= 0x03;

    // pg 496, STM32F411 ref manual
    // Bit 1-7: interface address is 100001 = 0x21
    // Bit 14: always keep to 1
    // 7 bit addressing, bit 15 --> 0
    I2C1->OAR1 |= ((0x21 << 1) | (1 << 14));

    // enable I2C1
    I2C1->CR1 |= 1;

    // STM32F411-E Disco has LSM303DLHC.  Accel. datasheet does not document
    // 0x07 addressed register WHO_AM_I, but LSM303D accel does. Return 8 bit constant
    // For LSM303DLHC, constant is 0x33.  Sanity check that everything working.
    unsigned char accelID = accelerometer_read(0xF);
    if (accelID != 0x33) {
        // error condition
        BSP_LED_redOn();
    }

    // pg 24, LSM303DLHC datasheet. CTRL_REG1_A is at address 0x20.
    // set CTRL_REG1_A to 0x47 to configure normal/low-power (50 MHz) and enable X, Y, Z axes
    accelerometer_write(0x20, 0x47);

    // set up magnetometer here
    // set CRA_REG_M data output to b110 (75 Hz min sample freq) --> b10011000 = 0x98
    //                                                               b10011100 = 0x9C
    // 0x00, reset value: 0x08
    magnetometer_write(0x00, 0x9C);

    // set CRB_REG_M gain bits to b111 (+/-8.1 gauss --> gain XYZ (230) gain z (205) --> b1110 0000 = 0xE0
    // output range: -2048 to 2047
    // 0x01, reset value: 0x10
    magnetometer_write(0x01, 0xE0);

    // orient magnetometer
    mag_init();
}   // END accelerometer_init


// accelGetValues
void accelGetValues(short* x, short* y, short* z) {
    // fill x
    *x = accelerometer_read(0x29) << 8;
    *x |= accelerometer_read(0x28);

    // fill y
    *y = accelerometer_read(0x2B) << 8;
    *y |= accelerometer_read(0x2A);

    // fill z
    *z = accelerometer_read(0x2D) << 8;
    *z |= accelerometer_read(0x2C);
}   // END accelGetValues


// https://www.hobbytronics.co.uk/accelerometer-info#:~:text=Measuring%20Tilt%20Angle%20using%20One,one%20axis%20of%20the%20accelerometer.
void accel_xy_calc(double x_val, double y_val, double z_val, double* roll, double* pitch) {
    // Using x, y, and z from accelerometer, calculate roll and pitch angles (in degrees)
    *roll  = (atan2(-y_val, z_val) * 180.0) / PI;
    *pitch = (atan2(x_val, sqrt(y_val * y_val + z_val * z_val)) * 180.0) / PI;
}   // END accel_xy_calc


























