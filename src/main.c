


#include "kernel.h"


void example_task1();
void example_task2();

// main function, performs some initialization and then launches the kernel
int main(void) {
    // kernel initialization
    kernelInit();

    // task specific initializations
    BSP_LED_Init();

    // for my personal development of this OS for controlling a quadcopter with PWM motors, need to comment out
    // USART2 usage and enable TIM9 (see pwm.c) in order to support all motors. (USART2 and TIM9 share PA2 pin)
    // this should definitely use a flag, but haven't gotten to that yet
    USART2_Init();

    // task initialization (!! should be checking Add Threads error conditions !!)
    kernelAddThreads(&example_task1, 1);
    kernelAddThreads(&example_task2, 1);

    // launch kernel (and tasks)
    kernelLaunch();

    // never reached, just suppresses compiler warning
    return 0;
}   // END main




void example_task1() {
    long counter1 = 0;
    while(1) {
       counter1++;
       if(counter1 % 5000 == 0) {
           counter1 = 0;
           BSP_LED_redToggle();
       }
       threadYield();
   }
}



void example_task2() {
    USART2_send_str("hello world");
    while (1) {
        BSP_LED_greenToggle();
        busy_wait(5);
        threadYield();
    }
}
