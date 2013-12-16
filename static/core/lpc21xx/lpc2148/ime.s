	.section .text
	#.global_interrupts_set, "ax", %progbits
	.code 32

        .global global_interrupts_set
        .type global_interrupts_set, function
# r0==0 disable IRQ
# r0==1 enable IRQ
global_interrupts_set:
        mrs r3, cpsr
        cmp r0, #0
        orreq r2, r3, #0x80
        bicne r2, r3, #0x80
        msr cpsr_c, r2
        ands r2, r3, #0x80
        moveq r0, #1
        movne r0, #0
        bx lr
