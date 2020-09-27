/*
 *  osKernel.c      Kernel implementation file
 *  Implement kernel functions
 *  Created by Andrew Stange
 */

/*
 *  Preprocessor Directives
 */
#include "osKernel.h"
#define SYSPRI3         (*((volatile uint32_t *)0xE000ED20))        // SysTick priority register
#define ICSR         (*((volatile uint32_t *)0xE000ED04))        // Interrupt controller




/*
 *  Struct Definitions
 */
// Thread Control Block
struct tcb{
    int32_t *stackPt;
    struct tcb *nextPt;
    uint32_t sleepTime;
    uint32_t blocked;
    uint32_t priority;
};

// Periodic Task Control Block
typedef void(*taskT)(void);
typedef struct{
    taskT task;
    uint32_t period;
    uint32_t  timeLeft;
} tcb_periodic;



/*
 * Global Variables
 */

// thread control variables
uint32_t MILLIS_PRESCALER;                              // number of clock cycles per ms.  Dependent on BUS_FREQ
typedef struct tcb tcbType;
tcbType tcbs[NUM_OF_THREADS];                           // array of threads
int32_t TCB_STACK[NUM_OF_THREADS][STACKSIZE];           // main memory --> each thread gets an individual stack
tcbType *currentPt;                                     // currently running thread
int8_t current_reset = 0;                               // 1 to set currentPt->stackPt to NULL (ready for another task)

tcbType periodic_tcb;
int32_t PERIODIC_STACK[PERIOD_STACK_SIZE];



/*
 *  Helper Function Declarations
 */
void osSchedulerLaunch(void);


/*
 *  Function Definitions
 */

// osKernelStackInit
void osKernelStackInit(int i) {
    // set up stack for the given thread.  Fill stack with gibberish pattern and set TCB stack pointer
    tcbs[i].stackPt =  &TCB_STACK[i][STACKSIZE -16];    // set SP to top of stack
    TCB_STACK[i][STACKSIZE -1] =  0x01000000;           // set xPSR register (set bit 24 high to indicate Thumb ISA)
	                                                    // do not set PC (filled in by add threads function)
    TCB_STACK[i][STACKSIZE-3] = (int32_t) osReturnHandler;  // set LR to return handler (remove task from scheduler)
    TCB_STACK[i][STACKSIZE-4] = 0x12121212;             // R12
    TCB_STACK[i][STACKSIZE-5] = 0x03030303;             // R3
    TCB_STACK[i][STACKSIZE-6] = 0x02020202;             // R2
    TCB_STACK[i][STACKSIZE-7] = 0x01010101;             // R1
    TCB_STACK[i][STACKSIZE-8] = 0x00000000;             // R0
    TCB_STACK[i][STACKSIZE-9] = 0x11111111;             // R11
    TCB_STACK[i][STACKSIZE-10] = 0x10101010;            // R10
    TCB_STACK[i][STACKSIZE-11] = 0x09090909;            // R9
    TCB_STACK[i][STACKSIZE-12] = 0x08080808;            // R8
    TCB_STACK[i][STACKSIZE-13] = 0x07070707;            // R7
    TCB_STACK[i][STACKSIZE-14] = 0x06060606;            // R6
    TCB_STACK[i][STACKSIZE-15] = 0x05050505;            // R5
    TCB_STACK[i][STACKSIZE-16] = 0x04040404;            // R4
}   // END osKernelStacKInit




// osKernelAddThreads       MUST CHECK RETURN VALUE
int8_t osKernelAddThreads( void(*task)(void), uint32_t priority ) {
    // critical section (avoid being interrupted by context switch)
	__disable_irq();

	// iterate through array of thread blocks.  Check for open block, else error  (signified by a NULL stack pointer)
    int open_block = -1;
	for(int i = 0; i < NUM_OF_THREADS; i++) {
        if( tcbs[i].stackPt == NULL ) {     // find first open thread block
            open_block = i;
            break;
        }
    }
	if( open_block == -1 ) {        // no open blocks, return error
	    return -1;
	}

	// initialize stack (sets stack pointer to appropriate address)
	osKernelStackInit(open_block);

	// unblock thread, set priority, and set sleep time to 0    (set all members of TCB to avoid random values)
    tcbs[open_block].blocked = 0;
    tcbs[open_block].priority = priority;
    tcbs[open_block].sleepTime = 0;

	// set task (sets PC on the thead stack to the task function pointer)
	TCB_STACK[open_block][STACKSIZE-2] = (int32_t)(task); /*Init PC*/
	__enable_irq();

    // successful function execution
	return 1;
}   // END osKernelAddThreads





// osKernelInit
void osKernelInit(void) {
    // prescalar for milliseconds, given the bus frequency (MHz --> milliseconds)
    MILLIS_PRESCALER = (BUS_FREQ/1000);

    // Frequency options :  1hz, 10hz, 100hz, 1KHz,10KHz, 100KHz
    // freq --> number of times/second to run the task
    // osPeriodicTask_Init(periodic_events_execute, 5);

    // set all threads as uninitialized and make tcbs array a circular buffer (link last task to first one)
    for(int i = 0; i < NUM_OF_THREADS; i++) {
        // do not let scheduler pick uninitialized threads.  These values will be overwritten if a task is inputted
        tcbs[i].stackPt = NULL;     // NULL stackPt signifies an available thread block
        tcbs[i].blocked = 1;        // block thread
        tcbs[i].priority = 255;     // set to lowest priority

        // set up nextPt (link all thread control blocks together)
        if( i != (NUM_OF_THREADS - 1) ) {
            tcbs[i].nextPt = &tcbs[i+1];
        }
        else {
            tcbs[i].nextPt = &tcbs[0];      // make TCB array a circular buffer
        }
    }

    // set current pointer to first task (will be the first task block to be filled by add threads function)
    currentPt = &tcbs[0];

    // enable active FPU register storage (during context switches)
    // FPU has not been activated yet, so can edit these bits
    FPU->FPCCR |= FPU_FPCCR_ASPEN_Msk;		// set CONTROL.FPCA to 1, preserve FP context automatically
    FPU->FPCCR &= ~FPU_FPCCR_LSPEN_Msk;	// disable lazy FPU context save

    // enable FPU through CPACR
    SCB->CPACR |= (3UL << 2*10) | (3UL << 2*11);
    // CPACR_CP_Msk_(11) | CPACR_CP_Msk_(10);		// will not compile this way
}   // END osKernelInit




// osKernelLaunch
void osKernelLaunch() {
    // SysTick set up
    SysTick->CTRL &= ~(1);                             // disable SysTick
    SysTick->VAL  = 0;                                 // reset SysTick counter
    SysTick->LOAD = ((uint32_t)QUANTA * MILLIS_PRESCALER)-1;     // num clock cycles per quanta
    SYSPRI3 = (SYSPRI3 & 0x00FFFFFF) | 0x07000000;     // set SysTick priority to 7
    SysTick->CTRL |=  0x7;                       // enable SysTick

    SYSPRI3 = (SYSPRI3 & 0xFF00FFFF) | 0x00080000;		// set PendSV priority to 8

    // launch scheduler
    osSchedulerLaunch();
}   // END osKernelLaunch



// osThreadYield
void osThreadYield(void){
    __disable_irq();
    SysTick->CTRL &= ~(1);                             // disable SysTick
    SysTick->VAL  = 0;                                 // reset SysTick counter
    SysTick->LOAD = ((uint32_t)QUANTA * MILLIS_PRESCALER)-1;     // num clock cycles per quanta
    SysTick->CTRL |=  0x7;                              // enable SysTick
    __enable_irq();

    ICSR |= 0x04000000;       // trigger SysTick (pg 655, ARM v7M ref manual)
}   // END osThreadYield




// SysTick_Handler
void SysTick_Handler(void){
   ICSR  |=  0x10000000;     // trigger PendSV (pg 655, ARM v7M ref manual)
}   // END SysTick_Handler




// osPriorityScheduler
void osPriorityScheduler(void) {
    // This function is called from the PendSV_Handler (in assembly).  Set currentPt to the next thread to run set up
    tcbType* thread_ptr = currentPt;            // used to iterate through buffer of thread blocks
  	tcbType* nextThreadToRun = thread_ptr;
  	uint8_t highestPriorityFound = 255;         // min priority

  	// if currentPt has just terminated (reset boolean is set), set stackPt to NULL
  	if( current_reset == 1 ) {
  	    currentPt->stackPt = NULL;              // mark this thread block as available again
  	    current_reset = 0;
  	}

    // find highest priority thread that is not blocked or sleeping
    do{
        thread_ptr = thread_ptr->nextPt;
        if((thread_ptr->priority < highestPriorityFound) && (thread_ptr->blocked == 0) && (thread_ptr->sleepTime == 0)){
            nextThreadToRun = thread_ptr;
            highestPriorityFound = thread_ptr->priority;
        }
    } while(thread_ptr != currentPt);           // iterate through entire circular buffer of tasks

    // assembly code will perform context switch when this function ends
    currentPt = nextThreadToRun;

  	// return to PendSV handler in order to complete context switch
}   // END osPriorityScheduler








// osReturnHandler
void osReturnHandler(void) {
    // reset current task to empty defaults
    currentPt->blocked = 1;        // block thread
    currentPt->priority = 255;     // set to lowest priority

    // mark this thread block for removal.  stackPt set to NULL by scheduler to avoid issues with
    // having a NULL stack to save state to in the context switch following osThreadYield call
    current_reset = 1;

    // yield rest of quanta to scheduler (this task will be ignored
    osThreadYield();
}   // END osReturnHandler





// My debugger is a POS and will only allow viewing of global variables when in hard fault
// HardFault_Handler
#define ACCESS(address)      *((volatile unsigned int*)(address))
// system control block
uint32_t hfsr = 0;			// see pg 669 of ARM v7M manual
uint32_t cfsr = 0;			// see pg 665 of ARM v7M manual
uint8_t mmsr = 0;
uint8_t bfsr = 0;			// see pg 667 of ARM v7M manual
uint32_t afsr = 0;

// FP + MISC
uint32_t stir = 0;			// see pg 675 of ARM v7M manual
uint16_t ufsr = 0;			// see pg 668 of ARM v7M manual
uint32_t cpacr = 0;			// see pg 670 of ARM v7M manual
uint32_t fpccr = 0;			// see pg 672 of ARM v7M manual
uint32_t shcsr = 0;			// see pg 663 of ARM v7M manual
uint32_t fpscr = 0;

// new
uint32_t icsr = 0;          // see pg 227 of Cortex M4 generic user guide
uint32_t cpuid = 0;
uint32_t shpr1 = 0;
uint32_t shpr2 = 0;
uint32_t shpr3 = 0;
void HardFault_Handler(void) {
    // log register values, viewable in debugger (ASM below trigger breakpoint)
    hfsr = ACCESS(0xE000ED2C);
    afsr = ACCESS(0xE000ED3C);
    mmsr = ACCESS(0xE000ED28);
    bfsr = ACCESS(0xE000ED38);
    stir = ACCESS(0xE000EF00);
    cfsr = ACCESS(0xE000ED28);
    ufsr = ACCESS(0xE000ED2A);
    cpacr = ACCESS(0xE000ED88);
    fpccr = ACCESS(0xE000EF34);
    shcsr = ACCESS(0xE000ED24);
    fpscr = __get_FPSCR();

    icsr = ACCESS(0xE000ED04);          // see pg 227 of Cortex M4 generic user guide
    cpuid = ACCESS(0xE000ED00);
    shpr1 = ACCESS(0xE000ED18);
    shpr2 = ACCESS(0xE000ED1C);
    shpr3 = ACCESS(0xE000ED20);

    // __ASM volatile("BKPT #01");      // uncomment to trigger debugger breakpoint automatically here
    while(1) {
    }
}       // END HardFault_Handler


