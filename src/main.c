/*
 *  main.c      Main function
 *  Initialize and start the kernel
 */


#include "osKernel.h"


void uart_print(void) {
    USART2_send_str("Hello world!\r\n");
    int16_t g_x, g_y, g_z;
    int16_t a_x, a_y, a_z;
    while (1) {
        BSP_LED_greenToggle();
        gyroGetValues(&g_x, &g_y, &g_z);
        accelGetValues(&a_x, &a_y, &a_z);

        // gyro output
        USART2_send_str("g_x: ");
        USART2_send_int(g_x);
        USART2_send_str("\tg_y: ");
        USART2_send_int(g_y);
        USART2_send_str("\tg_z: ");
        USART2_send_int(g_z);
        USART2_send_str("\r\n");

        // accelerometer output
        USART2_send_str("a_x: ");
        USART2_send_int(a_x);
        USART2_send_str("\ta_y: ");
        USART2_send_int(a_y);
        USART2_send_str("\ta_z: ");
        USART2_send_int(a_z);
        USART2_send_str("\r\n");
        USART2_send_str("\r\n");
        osThreadYield();
    }
}




void LED_task(void) {
    while(1) {
        BSP_LED_blueToggle();
        BSP_LED_orangeToggle();
        BSP_LED_redToggle();
        osThreadYield();
    }
}



// main function
int main(void) {
    // kernel initialization
    osKernelInit();

    // task specific initializations
    BSP_LED_Init();
    USART2_Init();
    gyroInit();
    accelerometer_init();

    // task initialization
    osKernelAddThreads(&LED_task, 1);
    osKernelAddThreads(&uart_print, 1);
    osKernelLaunch();
}   // END main



