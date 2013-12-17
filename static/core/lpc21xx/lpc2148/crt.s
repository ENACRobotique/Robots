/* ***************************************************************************************************************

	crt.s						STARTUP  ASSEMBLY  CODE
								-----------------------


	Module includes the interrupt vectors and start-up code.

  *************************************************************************************************************** */

/* Stack Sizes */
.set  UND_STACK_SIZE, 0x00000010		/* stack for "undefined instruction" interrupts is 16 bytes */
.set  ABT_STACK_SIZE, 0x00000010		/* stack for "abort" interrupts is 16 bytes                 */
.set  FIQ_STACK_SIZE, 0x00000010		/* stack for "FIQ" interrupts  is 16 bytes         			*/
.set  IRQ_STACK_SIZE, 0X00000100		/* stack for "IRQ" normal interrupts is 256 bytes    		*/
.set  SVC_STACK_SIZE, 0x00000010		/* stack for "SVC" supervisor mode is 16 bytes  			*/

/* Standard definitions of Mode bits and Interrupt (I & F) flags in PSRs (program status registers) */
.set  MODE_USR, 0x10            		/* Normal User Mode 										*/
.set  MODE_FIQ, 0x11            		/* FIQ Processing Fast Interrupts Mode 						*/
.set  MODE_IRQ, 0x12            		/* IRQ Processing Standard Interrupts Mode 					*/
.set  MODE_SVC, 0x13            		/* Supervisor Processing Software Interrupts Mode 			*/
.set  MODE_ABT, 0x17            		/* Abort Processing memory Faults Mode 						*/
.set  MODE_UND, 0x1B            		/* Undefined Processing Undefined Instructions Mode 		*/
.set  MODE_SYS, 0x1F            		/* System Running Priviledged Operating System Tasks  Mode	*/

.set  I_BIT, 0x80               		/* when I bit is set, IRQ is disabled (program status registers) */
.set  F_BIT, 0x40               		/* when F bit is set, FIQ is disabled (program status registers) */


.text
.arm
.section ".init"
.global	reset_handler
.global _startup
.func   _startup

_startup:

# Exception Vectors

_vectors:       ldr     PC, Reset_Addr
                ldr     PC, Undef_Addr
                ldr     PC, SWI_Addr
                ldr     PC, PAbt_Addr
                ldr     PC, DAbt_Addr
                .word 	0xb9205f84		/* Reserved Vector (holds Philips ISP checksum) */
                ldr     PC, [PC,#-0xFF0]	/* see page 71 of "Insiders Guide to the Philips ARM7-Based Microcontrollers" by Trevor Martin  */
                ldr     PC, FIQ_Addr

Reset_Addr:     .word   reset_handler
Undef_Addr:     .word   undef_handler
SWI_Addr:       .word   swi_handler
PAbt_Addr:      .word   pabort_handler
DAbt_Addr:      .word   dabort_handler
IRQ_Addr:       .word   irq_handler
FIQ_Addr:       .word   fiq_handler
                .word   0					/* rounds the vectors and ISR addresses to 64 bytes total  */


# Reset Handler

reset_handler:
                /* Setup clock source : Fosc = 12Mhz, P = 2, M = 5 : Fcco = 240Mhz, cclk = 60Mhz */
                ldr r0, =0xE01FC000
                /* Configure PLL Multiplier/Divider */
                ldr r1, =0x24
                str r1, [r0, #0x084]
                /* Enable PLL */
                mov r1, #0x1
                str r1, [r0, #0x080]
                mov r1, #0xAA
                str r1, [r0, #0x08C]
                mov r1, #0x55
                str r1, [r0, #0x08C]
                /* Wait for PLL to lock */
pll_lock_loop:
                ldr r1, [r0, #0x088]
                tst r1, #0x400
                beq pll_lock_loop
                /* PLL Locked, connect PLL as clock source */
                mov r1, #0x3
                str r1, [r0, #0x080]
                mov r1, #0xAA
                str r1, [r0, #0x08C]
                mov r1, #0x55
                str r1, [r0, #0x08C]

                /* Initialise memory accelerator module */
                mov r1, #0
                str r1, [r0, #0x000]
                ldr r1, =3
                str r1, [r0, #0x004]
                ldr r1, =2
                str r1, [r0, #0x000]

                /* Configure the VPB clock */
                ldr r1, =2
                str r1, [r0, #0x100]

				/* Setup a stack for each mode - note that this only sets up a usable stack
				for User mode.   Also each mode is setup with interrupts initially disabled. */

    			mrs   r0, CPSR
    			bic   r0, r0, #0x1f
    			ldr   r2, =_stack_end
    			/* Undefined Istruction Mode */
    			orr   r1, r0, #MODE_UND|I_BIT|F_BIT
    			msr   CPSR_cxsf, r1
    			mov   sp, r2
    			sub   r2, r2, #UND_STACK_SIZE
    			/* Abort Mode */
                orr   r1, r0, #MODE_ABT|I_BIT|F_BIT
                msr   CPSR_cxsf, r1
                mov   sp, r2
                sub   r2, r2, #ABT_STACK_SIZE
                /* FIQ Mode */
                orr   r1, r0, #MODE_FIQ|I_BIT|F_BIT
                msr   CPSR_cxsf, r1
                mov   sp, r2
                sub   r2, r2, #FIQ_STACK_SIZE
                /* IRQ Mode */
                orr   r1, r0, #MODE_IRQ|I_BIT|F_BIT
                msr   CPSR_cxsf, r1
                mov   sp, r2
                sub   r2, r2, #IRQ_STACK_SIZE
                /* Supervisor Mode */
                orr   r1, r0, #MODE_SVC|I_BIT|F_BIT
                msr   CPSR_cxsf, r1
                mov   sp, r2
                sub   r2, r2, #SVC_STACK_SIZE
                /* System Mode */
                orr   r1, r0, #MODE_SYS|I_BIT|F_BIT
                msr   CPSR_cxsf, r1
                mov   sp, r2


				/* copy .data section (Copy from ROM to RAM) */
                ldr     R1, =_etext
                ldr     R2, =_data
                ldr     R3, =_edata
1:        		cmp     R2, R3
                ldrlo   R0, [R1], #4
                strlo   R0, [R2], #4
                blo     1b

				/* Clear .bss section (Zero init)  */
                mov     R0, #0
                ldr     R1, =_bss_start
                ldr     R2, =_bss_end
2:				cmp     R1, R2
                strlo   R0, [R1], #4
                blo     2b


				/* Enter the C code  */
                b       main
.endfunc

undef_handler:
  b undef_handler

swi_handler:
  b swi_handler

pabort_handler:
  b pabort_handler

dabort_handler:
  b dabort_handler

fiq_handler:
  b fiq_handler

irq_handler:
  b irq_handler

  .weak undef_handler, swi_handler, pabort_handler, dabort_handler, fiq_handler, irq_handler

.end
