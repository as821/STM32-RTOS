


#include "kernel.h"


void pwm_task();
void uart_print();

// main function
int main(void) {
    // kernel initialization
    kernelInit();

    // task specific initializations
    BSP_LED_Init();
    gyroInit();
    accelerometer_init();        // calls magnetometer init

    // for my personal development of this OS for controlling a quadcopter with PWM motors, need to comment out
    // USART2 usage and enable TIM9 (see pwm.c) in order to support all motors. (USART2 and TIM9 share PA2 pin)
    // this should definitely use a flag, but haven't gotten to that yet
    USART2_Init();

    // task initialization (!! should be checking Add Threads error conditions !!)
    kernelAddThreads(&pwm_task, 1);
    kernelAddThreads(&uart_print, 1);

    // launch kernel (and tasks)
    kernelLaunch();
}   // END main




// PWM task
long counter1 = 0;
void pwm_task() {
    while(1) {
       counter1++;
       if(counter1 % 15 == 0) {
           counter1 = 0;
           BSP_LED_orangeToggle();
       }
       threadYield();
   }
}   // END pwm_task



// uart_print
void uart_print() {
    USART2_send_str("hello world");

    while (1) {
        BSP_LED_greenToggle();
        busy_wait(1);       // in seconds
        threadYield();
    }
}   // END uart_print
