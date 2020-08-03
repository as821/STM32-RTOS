# STM32-RTOS
A Mininal Real Time Operating System for the STM32F411

Written using CMSIS register address header files for the STM32.  No external libraries or HAL are used.  All memory allocated statically.

Uses a priority-based preemptive scheduler to coordinate task execution.  The operating system also provides semaphores as well as messaging and a FIFO queue for interprocess communication.

Project includes drivers for:
* Gyroscope
* Accelerometer
* USART2 (with and without DMA TX)
* USART1 (with DMA RX)
* ADC
* I2C
* SPI

Written to support quadcopter flight control software on the STM32F411VE.

