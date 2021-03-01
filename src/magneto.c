/*
 *  magneto.c      Magnetometer Driver
 *  Magnetometer driver.  Uses same I2C interface as accelerometer
 *  Created by Andrew Stange
 */

#include "osKernel.h"
#define MAGNETOMETER_READ  0x3D
#define MAGNETOMETER_WRITE 0x3C


// mag_init
void mag_init(void)  {
    mag_orientation();
}   // END mag_init




// magnetometer_read
unsigned char magnetometer_read(unsigned short addr) {
    // see pg 19, LSM303DLHC datasheet. Details interaction with sensor over I2C
    I2C1_busy_wait();
    I2C1_enable_ack();
    I2C1_restart();

    I2C1_send_slave_addr(MAGNETOMETER_WRITE);
    I2C1_send_reg(addr);
    I2C1_restart();

    I2C1_send_slave_addr(MAGNETOMETER_READ);
    I2C1_disable_ack();
    I2C1_stop();

    return I2C1_get_data();
}   // END magnetometer_read




// magnetometer_write
void magnetometer_write(unsigned short addr, unsigned char data) {
    // see pg 19, LSM303DLHC datasheet. Details interaction with sensor over I2C
    I2C1_busy_wait();
    I2C1_restart();

    I2C1_send_slave_addr(MAGNETOMETER_WRITE);
    I2C1_send_reg(addr);
    I2C1_write_byte(data);

    I2C1_stop();
}   // END magnetometer_write






// magGetValues
uint8_t magGetValues(int16_t* x, int16_t* y, int16_t* z) {
    BSP_LED_blueOn();
    if(magnetometer_read(0x09) & 0x1) {     // check status register (SR_REG_M) for "data ready" bit
        *x = (magnetometer_read(0x03) << 8 | magnetometer_read(0x04));
        *y = (magnetometer_read(0x07) << 8 | magnetometer_read(0x08));
        *z = (magnetometer_read(0x05) << 8 | magnetometer_read(0x06));
        BSP_LED_blueOff();
        return 1;
    }
    else {
        BSP_LED_blueOff();
        return 0;       // error condition, new data not ready
    }
}   // END magGetValues





double orient_heading;
// mag_orientation
void mag_orientation(void) {
    BSP_LED_redOn();
    int16_t x, y, z;

    // give mag LPF time to settle to default value
    magnetometer_write(0x02, 0x00);     // continuous conversion (needed for mag. to work)
    busy_wait(0.001f);                          // fails without this, think just needs time to switch
    for (int i = 0; i < MAG_ORIENT; i++) {
        // sensor reading
        magGetValues(&x, &y, &z);

        // get heading
        orient_heading = mag_heading_calc(x, y, z);
    }
    magnetometer_write(0x02, 0x03);     // single conversion (needed for accel. to work)
    BSP_LED_redOff();
}   // END mag_orientation




// mag_heading_calc
double mag_heading_calc(int16_t x_val, int16_t y_val, int16_t z_val) {
    return atan2(y_val, x_val) * 180/PI;
}   // END mag_heading_calc



// mag_angle_calc
double mag_angle_calc(double cur_heading) {
    return orient_heading - cur_heading;
}   // END mag_angle_calc







