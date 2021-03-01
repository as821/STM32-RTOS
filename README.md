# STM32-RTOS
A Mininal Real Time Operating System for the STM32F411

Written using CMSIS register address header files for the STM32.  No external libraries or HAL are used.  All memory allocated statically.

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

Written to support quadcopter flight control software on the STM32F411VET6.  OS code size is about 70KB, CMSIS header files take up ~750KB.  Size of OS when running depends entirely on the number of threads and the stack size.


NOTE: code style is still a bit rough around the edges, since this is just a snapshot mid-development.  From around this point, I forked off to work on quadcopter specific stuff (stabilization, motor control, bluetooth connection to controller, etc.) without significant changes to the OS code.  Although not the cleanest, the code here is pretty short and straightforward.  At some point, I plan to switch over to using the CMSIS defined constants more consistently throughout the code. 

Honestly, haven't had time to work on this due to graduate school applications, etc.  This summer maybe??  Either way,  this is my first significant foray into the world of bare metal embedded OS, enjoy :)


