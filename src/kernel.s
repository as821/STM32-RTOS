;
;  kernel.s      Kernel assembly file
;  Implement fast context switching in assembly
;  Created by Andrew Stange
;
			AREA |.text|,CODE,READONLY,ALIGN=2
			THUMB
			PRESERVE8
			EXTERN currentPt                ; actually &currentPt, not currentPt itself
		    EXPORT PendSV_Handler
			EXPORT schedulerLaunch
			IMPORT priorityScheduler


; handle PendSV interrupt.  Triggered by SysTick.  Used to enact a context switch between threads
PendSV_Handler                      ; hardware saves r0,r1,r2,r3,r12,lr,pc,psr just by calling this function
	; save state of thread being suspended
	CPSID	  I                     ; disable interrupts
	PUSH 	  {R4-R11}              ; store R4-R11 on stack
	LDR 	  R0,=currentPt         ; R0 = &currentPt
	LDR		  R1,[R0]               ; R1 = currentPt
	STR 	  SP,[R1]               ; currentPt->stackPt = SP --> store current stack pointer in TCB

    ; determine which thread to run next
    PUSH	  {R0,LR}
	BL		  priorityScheduler   ; call priorityScheduler --> select next thread to run
	POP		  {R0,LR}

	; load TCB of next thread to run
	LDR 	  R1,[R0]               ; R1 = next thread to run (determined in C scheduler function)
	LDR 	  SP,[R1]               ; set stack pointer for next thread to run
	POP		  {R4-R11}              ; restore R4-R11 from that function's stack
	CPSIE	  I                     ; enable interrupts
	BX		  LR                    ; exit function
; END PendSV_Handler




; start the scheduler (initial context switch into first thread to run)
schedulerLaunch
    ; set stack pointer to value for first thread
	LDR		R0,=currentPt           ; R0 = &current_Pt
	LDR		R2,[R0]                 ; R2 = current_Pt
	LDR		SP,[R2]                 ; SP = current_Pt->stackPt
	POP		{R4-R11}                ; restore registers from this stack
	POP		{R0-R3}
	POP 	{R12}

	ADD		SP,SP,#4                ; skip over link register value stored on stack
	POP		{LR}                    ; load LR with PC value stored on stack (function pointer of first thread task)
	ADD		SP,SP,#4                ; skip PSR value on stack
	CPSIE    I                      ; enable interrupts
	BX		 LR                     ; return from subroutine
; END schedulerLaunch

	ALIGN
	END