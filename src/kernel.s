//
//  kernel.s      Kernel assembly file
//  Implement fast context switching in assembly
//  Created by Andrew Stange
//
//  Conversion from Legacy ARM assembly (armasm) to GNU assembly syntax (armclang/as):
//  https://developer.arm.com/documentation/dui0742/k/Migrating-from-armasm-to-the-armclang-Integrated-Assembler/Overview-of-differences-between-armasm-and-GNU-syntax-assembly-code?lang=en
//


			//AREA |.text|,CODE,READONLY,ALIGN=2
            .section .text,"x"
            .balign 4
			//THUMB
            .thumb
            .syntax unified
			//PRESERVE8
            .eabi_attribute Tag_ABI_align_preserved, 1



			// EXTERN currentPt                // actually &currentPt, not currentPt itself
            // TODO EXTERN command is not addressed in migration, try .global or .extern for now??
            .extern currentPt




			//IMPORT priorityScheduler
            .global priorityScheduler     // need a .type here??



// handle PendSV interrupt.  Triggered by SysTick.  Used to enact a context switch between threads
//EXPORT PendSV_Handler
.global PendSV_Handler
.type PendSV_Handler STT_FUNC // https://developer.arm.com/documentation/dui0742/k/Migrating-from-armasm-to-the-armclang-Integrated-Assembler/Miscellaneous-directives?lang=en
PendSV_Handler:                      // hardware saves r0,r1,r2,r3,r12,lr,pc,psr just by calling this function
	// save state of thread being suspended
	CPSID	  I                     // disable interrupts
	PUSH 	  {R4-R11}              // store R4-R11 on stack
	LDR 	  R0,=currentPt         // R0 = &currentPt
	LDR		  R1,[R0]               // R1 = currentPt
	STR 	  SP,[R1]               // currentPt->stackPt = SP --> store current stack pointer in TCB

    // determine which thread to run next
    PUSH	  {R0,LR}
	BL		  priorityScheduler   // call priorityScheduler --> select next thread to run
	POP		  {R0,LR}

	// load TCB of next thread to run
	LDR 	  R1,[R0]               // R1 = next thread to run (determined in C scheduler function)
	LDR 	  SP,[R1]               // set stack pointer for next thread to run
	POP		  {R4-R11}              // restore R4-R11 from that function's stack
	CPSIE	  I                     // enable interrupts
	BX		  LR                    // exit function
// END PendSV_Handler




// start the scheduler (initial context switch into first thread to run)
//EXPORT schedulerLaunch
.global schedulerLaunch
.type schedulerLaunch STT_FUNC      // not clear if this should be .type or .text ...
schedulerLaunch:
    // set stack pointer to value for first thread
	LDR		R0,=currentPt           // R0 = &current_Pt
	LDR		R2,[R0]                 // R2 = current_Pt
	LDR		SP,[R2]                 // SP = current_Pt->stackPt
	POP		{R4-R11}                // restore registers from this stack
	POP		{R0-R3}
	POP 	{R12}

	ADD		SP,SP,#4                // skip over link register value stored on stack
	POP		{LR}                    // load LR with PC value stored on stack (function pointer of first thread task)
	ADD		SP,SP,#4                // skip PSR value on stack
	CPSIE    I                      // enable interrupts
	BX		 LR                     // return from subroutine
// END schedulerLaunch

    // ALIGN
    .balign         // this might be invalid, not sure this carries over completely from ARM asm
    // END
    .end
