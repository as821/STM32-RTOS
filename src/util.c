/*
 *  util.c      Random utilities and semaphores
 *  Random utilities and Semaphores
 *  Created by Andrew Stange
 */
#include "osKernel.h"

// buf_avg (assumes SENS_BUF_LEN)
double buf_avg(int16_t* buf, uint16_t len) {
    double sum = 0;
    for(int i = 0; i < len; i++) {
         sum += buf[i];
    }
    return sum / len;
}   // END buf_avg


// busy_wait
void busy_wait(float t) {
    // t in seconds, BUS_FREQ in ticks/sec;
    for(int i = 0; i < (int)(t * BUS_FREQ); i++);
}   // END busy_wait


// map
double map(double x, int16_t in_min, int16_t in_max, int16_t out_min, int16_t out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}   // END map




// to_degree
double to_degree(double val) {
    return val * (180 / PI);
}   // END to_degree




// https://kiritchatterjee.wordpress.com/2014/11/10/a-simple-digital-low-pass-filter-in-c/
// in-place IIR low pass filter
void iir_lpf(int16_t* buf, uint16_t len, float beta, double* smooth) {
    for(int i = 0; i < len; i++) {
        if(i != 0) {
            buf[i] = buf[i-1] - (beta * (buf[i-1] - buf[i]));
        }
        else {
            buf[0] = *smooth - (beta * (*smooth - buf[0]));
        }
    }
    *smooth = buf[len - 1];
}   // END iir_lpf






// osSemaphoreInit
void semaphore_init(int32_t *semaphore,int32_t value) {
    *semaphore = value;
}   // END osSemaphoreInit





// osSignalSet
void semaphore_set(int *semaphore) {
    __disable_irq();
    *semaphore += 1;       // increment semaphore value to set it as open
    __enable_irq();
}   // END osSignalSet





// semaphore_wait
void semaphore_wait(int32_t *semaphore) {
    // while semaphore is not available, yield rest of quanta (avoid wasting CPU)
    __disable_irq();
    while( *semaphore <= 0 ) {
        __enable_irq();
        osThreadYield();
        __disable_irq();
    }
    *semaphore -= 1;           // acquire the semaphore and return
    __enable_irq();
}   // END semaphore_wait



