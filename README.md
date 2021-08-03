# STM32-RTOS
A Mininal Real Time Operating System for the STM32F411

Written in C using CMSIS register address header files for the STM32.  No C stdlib or HAL are used.  All memory allocated statically.

Uses a priority-based preemptive scheduler to coordinate task execution.  The operating system also provides semaphores, a mailbox, and a FIFO queue for interprocess communication.

Project includes drivers for:
* Gyroscope (on board STM32F411VET6)
* Accelerometer (on board STM32F411VET6)
* Magnetometer (on board STM32F411VET6)
* USART2 (with and without DMA TX)
* USART1 (polling)
* ADC
* I2C
* SPI

Written to support quadcopter flight control software on the STM32F411VET6.

NOTE: the code style in this repo is a little rough around the edges since this is a mid-development snapshot.  This is the state of the OS code when I forked off to work on quadcopter specific code (stabilization, motor control, bluetooth connection to controller, etc.).  Due to graduate school applications, etc. I have not had as much time to work on this in the past year but I am making some edits here and there as I have time.

Either way,  this is my first significant foray into the world of bare metal embedded OS, enjoy :)




TODO:
* Add a check for saving FPU state in the context switch assembly code
* Add an "isb" command when exiting ISRs so pipline is flushed before execution leaves the function
* Use a different data structure/algorithm for the priority scheduler to speed up context switches.

