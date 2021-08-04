//
//  kernel.s      Kernel assembly file
//  Implement fast context switching in assembly
//  Created by Andrew Stange
//
//  Conversion from Legacy ARM assembly (armasm) to GNU assembly syntax (armclang/as):
//  https://developer.arm.com/documentation/dui0742/k/Migrating-from-armasm-to-the-armclang-Integrated-Assembler/Overview-of-differences-between-armasm-and-GNU-syntax-assembly-code?lang=en
//  GNU asm syntax: https://sourceware.org/binutils/docs/as/index.html#SEC_Contents
//  ARM asm syntax: https://www.keil.com/support/man/docs/armasm/armasm_dom1361290000455.htm


            .text                                           // .text sections are executable code sections
            .balign 4                                       // align on 4-byte boundary

            .thumb
            .syntax unified
            .eabi_attribute Tag_ABI_align_preserved, 1      // PRESERVE8 in legacy ARM asm, tell assembler that we
                                                            // preserve 8 byte stack alignment
                                                            // https://developer.arm.com/documentation/100070/0608/gap1499870365595

            .extern currentPt               // actually &currentPt, not currentPt itself
            .global priorityScheduler       // this is a C function defined elsewhere, allow the linker to find it



// handle PendSV interrupt.  Triggered by SysTick.  Used to enact a context switch between threads
.global PendSV_Handler
.type PendSV_Handler STT_FUNC           // https://developer.arm.com/documentation/dui0742/k/Migrating-from-armasm-to-the-armclang-Integrated-Assembler/Miscellaneous-directives?lang=en
PendSV_Handler:                         // hardware saves r0,r1,r2,r3,r12,lr,pc,psr just by calling this function
	// save state of thread being suspended
	cpsid	  I                     // disable interrupts
	push 	  {R4-R11}              // store R4-R11 on stack
	ldr 	  R0,=currentPt         // R0 = &currentPt
	ldr		  R1,[R0]               // R1 = currentPt
	str 	  SP,[R1]               // currentPt->stackPt = SP --> store current stack pointer in TCB

    // determine which thread to run next
    push	  {R0,LR}
    dsb                             // data and instruction barrier instructions
    isb
	bl		  priorityScheduler   // call priorityScheduler --> select next thread to run
	pop		  {R0,LR}

	// load TCB of next thread to run
	ldr 	  R1,[R0]               // R1 = next thread to run (determined in C scheduler function)
	ldr 	  SP,[R1]               // set stack pointer for next thread to run
	pop		  {R4-R11}              // restore R4-R11 from that function's stack
	dsb                             // data and instruction barrier instructions
    isb
	cpsie	  I                     // enable interrupts
	bx		  LR                    // exit function
// END PendSV_Handler




// start the scheduler (initial context switch into first thread to run)
.global schedulerLaunch
.type schedulerLaunch STT_FUNC      // not clear if this should be .type or .text ...
schedulerLaunch:
    // set stack pointer to value for first thread
	ldr		R0,=currentPt           // R0 = &current_Pt
	ldr		R2,[R0]                 // R2 = current_Pt
	ldr		SP,[R2]                 // SP = current_Pt->stackPt
	pop		{R4-R11}                // restore registers from this stack
	pop		{R0-R3}
	pop 	{R12}

	add		SP,SP,#4                // skip over link register value stored on stack
	pop		{LR}                    // load LR with PC value stored on stack (function pointer of first thread task)
	add		SP,SP,#4                // skip PSR value on stack
	cpsie    I                      // enable interrupts
	bx		 LR                     // return from subroutine
// END schedulerLaunch

    .balign 4           // in ARM asm, this was ALIGN with the AREA directive having an ALIGN expression value of 2,
                        // causing this to be aligned on a 2^2 == 4 byte boundary
    .end
