/*
 *  util.c      Random utilities, semaphores, mailbox, FIFO queue
 *  Random utilities, semaphores, mailbox, FIFO queue
 *  Created by Andrew Stange
 */
#include "kernel.h"

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



// semaphore_init
void semaphore_init(int32_t *semaphore,int32_t value) {
    *semaphore = value;
}   // END semaphore_init



// semaphore_set
void semaphore_set(int *semaphore) {
    __disable_irq();
    *semaphore += 1;       // increment semaphore value to set it as open
    __enable_irq();
}   // END semaphore_set


// semaphore_wait
void semaphore_wait(int32_t *semaphore) {
    // while semaphore is not available, yield rest of quanta (avoid wasting CPU)
    __disable_irq();
    while( *semaphore <= 0 ) {      // wait while semaphore is 0 or negative
        __enable_irq();
        threadYield();
        __disable_irq();
    }
    *semaphore -= 1;           // acquire the semaphore and return
    __enable_irq();
}   // END semaphore_wait





// mailbox_init
double mail_data;
int32_t mail_sem = 1;
int mail_has_data = 0;
void mailbox_init(void){
    mail_has_data = 0;                         // no data in mailbox
    mail_data  = 0;
    semaphore_init(&mail_sem,0);        // mailbox semaphore is taken
}   // END mailbox_init



// mailbox_write
void mailbox_write(uint32_t data){
    __disable_irq();
    if(mail_has_data) {                        // if data already in mailbox, do not overwrite
        __enable_irq();
        return;
    }
    mail_data = data;                         // place data in mailbox and set flag
    mail_has_data = 1;
    __enable_irq();
    semaphore_set(&mail_sem);                   // set semaphore as free (contains data)
}   // END mailbox_write




// mailbox_recv
uint32_t mailbox_recv(void){
    // wait until semaphore set as free by send side
    semaphore_wait(&mail_sem);
    uint32_t data;

    __disable_irq();
    data = mail_data;
    mail_has_data = 0;
    __enable_irq();

    // return data from mailbox
    return data;
}   // END mailbox_recv



// fifo_init
int put = 0;
int get = 0;
int os_fifo[FIFO_SIZE];
int current_fifo_size = 0;

void fifo_init(void) {
    __disable_irq();
    put = 0;
    get = 0;
    for(int i = 0; i < FIFO_SIZE; i++) { os_fifo[i] = 0; }      // zero FIFO buffer
    semaphore_init(&current_fifo_size,0);      // no data in queue
    lost_data = 0;
    __enable_irq();
}   // END fifo_init




// fifo_put
int fifo_put(uint32_t data){
    // if FIFO full, return error
    if(current_fifo_size == FIFO_SIZE) {
        lost_data++;
        return -1;
    }

    // else, place data in FIFO, critical section
    __disable_irq();
    os_fifo[put] = data;
    put = (put + 1) % FIFO_SIZE;
    semaphore_set(&current_fifo_size);        // signal that there is data in the FIFO queue
    __enable_irq();
    return 1;
}   // END fifo_put



// fifo_get
int32_t fifo_get(void) {
    // wait until there is data in the queue
    uint32_t data;
    semaphore_wait(&current_fifo_size);

    __disable_irq();
    data = os_fifo[get];
    get = (get + 1) % FIFO_SIZE;
    __enable_irq();

    return data;
}   // END fifo_get


