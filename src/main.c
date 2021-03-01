


#include "osKernel.h"

// PWM task
long counter1 = 0;
void pwm_task(void) {
    while(1) {
       counter1++;
       if(counter1 % 15 == 0) {
           counter1 = 0;
           BSP_LED_orangeToggle();
       }
       osThreadYield();
   }
}   // END pwm_task



// uart_print
void uart_print(void) {
    USART2_send_str("hello world");

    while (1) {
        BSP_LED_greenToggle();
        busy_wait(1);       // in seconds
        osThreadYield();
    }
}   // END uart_print





// main function
int main(void) {
   // kernel initialization
   osKernelInit();

   // task specific initializations
   BSP_LED_Init();
   gyroInit();
   accelerometer_init();        // calls magnetometer init

   /*
    * Once done with debugging, remove this USART2_Init and any USART2 function usage.  Uncomment TIM9 GPIO set up and
    * TIM9 set up in pwm as well (USART2 and TIM9 share PA2)
    */
   USART2_Init();


   // task initialization (!! should be checking Add Threads error conditions !!)
   osKernelAddThreads(&pwm_task, 1);
   osKernelAddThreads(&uart_print, 1);

   // launch kernel (and tasks)
   osKernelLaunch();
}   // END main




