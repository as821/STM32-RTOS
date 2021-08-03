/*
 *  imu.c      IMU function implementation
 *  Read data from onboard gyroscope
 *  Created by Andrew Stange
 */

#include "kernel.h"


// SPI1_RX_wait
void SPI1_RX_wait(void) {
    // pg 604, STM32F411 ref manual
    // loop while RX buffer is empty or while SPI is busy (in communication)
    while((SPI1->SR & 1) == 0 || (SPI1->SR & (1 << 7)) == 1) {}
}   // END SPI1_RX_wait




// SPI1_TX_wait
void SPI1_TX_wait(void) {
    // pg 604, STM32F11 ref manual
    // loop while TX buffer is not empty or while SPI is busy (in communication)
    while((SPI1->SR & (1 << 1)) == 0 || (SPI1->SR & (1 << 7)) == 1) {}
}   // END SPI1_TX_wait





// gyroRead
unsigned char gyroRead(unsigned char gyroRegister) {
    // reset PE3 (CS line for gyro), set CS low to select this chip
    // pg 160 of STM32F411 ref manual
    GPIOE->BSRR |= (1 << 19);

    SPI1_TX_wait();

    // see pg 26, L3GD20 datasheet.  First bit of transmission must be a 1 for READ
    SPI1->DR = (gyroRegister | 0x80);

    SPI1_RX_wait();
    SPI1->DR;
    SPI1_TX_wait();

    SPI1->DR = 0xFF;
    SPI1_RX_wait();

    // read from gyro
    unsigned char gyro_val = (unsigned char) SPI1->DR;

    // set CS to high, transmission ended
    GPIOE->BSRR |= (1 << 3);
    return gyro_val;
}   // END gyroRead





// gyroWrite
void gyroWrite(unsigned char gyroRegister, unsigned char value) {
    // PE3 is CS --> bring low to select slave device
    GPIOE->BSRR |= (1 << 19);

    // transmit sequence
    SPI1_TX_wait();
    SPI1->DR = gyroRegister;
    SPI1_RX_wait();
    SPI1->DR;
    SPI1_TX_wait();
    SPI1->DR = value;
    SPI1_RX_wait();
    SPI1->DR;

    // PE3 is CS --> return to high to deselect
    GPIOE->BSRR |= (1 << 3);
}   // END gyroWrite







// gyroInit
void gyroInit(void) {
    // Initialize gyro sensor measurement timer --> need regulated time diff. for integration
    TIM5_Init();

    // GPIO Port A clock enable (PA5-PA7 connected to gyro, pg 33 STM32F411VE user manual)
    RCC->AHB1ENR |= 1;

    // GPIO Port E enable (PE3 is chip select for gyro)
    RCC->AHB1ENR |= (1 << 4);

    // SPI1 clock enable
    RCC->APB2ENR |= (1 << 12);

    // Alternate function enable for PA5, PA6, PA7, PE3 (pg 20 STM32F411VE user manual)
    GPIOA->MODER |= (1 << 11) | (1 << 13) | (1 << 15);
    GPIOE->MODER |= (1 << 6);

    // pg 150, STM43F411 ref manual. SPI1 --> AF5
    // set PA5, PA6, PA7 to AF5.  5th, 6th and 7th blocks of 4 bits
    GPIOA->AFR[0] |= (5 << 20) | (5 << 24) | (5 << 28);

    // TODO need to set GPIOE AFR for PE3 --> to SPI1 (AF5) ??

    // set pins to fast speed (pg 157, STM32F411 ref manual)
    // 2 bits for each pin --> 5 is 10/11, 6 is 12/13, 7 is 14/15
    GPIOA->OSPEEDR |= (2 << 10) | (2 << 12) | (2 << 14);
    GPIOE->OSPEEDR |= (2 << 6);

    // SPI Control register configuration (pg 601 of STM32F411 ref manual)
    // bit 0 --> second clock transition is first data capture edge
    // bit 1 --> CK to 1 when idle
    // bit 2 --> STM32 is master, gyro is slave
    // bit 3-5 to 010 for baud rate of fPCLK / 8
    // bit 8-9 --> software slave management enable, internal slave select to 1
    SPI1->CR1 |= (1) | (1 << 1) | (1 << 2) | (2 << 3) | (1 << 8) | (1 << 9);

    // SPI1 enable (pg 601 of STM32F411 ref manual)
    SPI1->CR1 |= (1 << 6);

    // set PE3 to high (BS section of BSRR --> set ODR bit corresponding this bit to
    // high.  PE3 is CS for gyro, setting to low indicates communication)
    // See pg25 of L3GD20 datasheet for more info
    GPIOE->BSRR |= (1 << 3);


    // if set up correctly, should read b11010100 (0xD4) from 0xF (WHO_AM_I register)
    // pg 29, L3GD20 datasheet
    if(gyroRead(0x0F) != 0xD4) {
        // ERROR, init failed
    }

    // set to 2000 dps (highest sensitivity). reset value is 0x00
    gyroWrite(0x23,0x20);

    // power on gyro, enable X, Y, Z axis
    // pg 31, L3GD20 datasheet.  CTRL_REG1
    gyroWrite(0x20, 0x0F);
}   // END gyroInit





// getAxisValue
short getAxisValue(unsigned char lowRegister, unsigned char highRegister) {
    // pg 9, L3GD20 datasheet
    // default sensitivity, 250 dps (8.75 is corresponding value for sensitivity.  Scale by 0.001 (mdps --> dps)
    // read from gyro and convert to a +/- 360 degree value
    // see last post: https://www.avrfreaks.net/forum/gyro-raw-data-degrees-second
    float scaler = 0.07;        // 70 * 0.001 --> 2000dps gain value * mdps tp dps conversion
    int16_t temp = (gyroRead(lowRegister) | (gyroRead(highRegister) << 8));
    return (int16_t)((float) temp * scaler);
}   // END getAxisValue




// gyroGetValues
void gyroGetValues(short* x, short* y, short* z) {
    // pg 36 l3GD20 ref manual for addresses
    *x = getAxisValue(0x28, 0x29);
    *y = getAxisValue(0x2A, 0x2B);
    *z = getAxisValue(0x2C, 0x2D);
}   // END gyroGetValues



