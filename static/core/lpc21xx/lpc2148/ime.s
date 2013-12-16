	.section .text
	.code 32

    .global global_IRQ_set
    .type global_IRQ_set, function
# r0==0 disable IRQ
# r0==1 enable IRQ
global_IRQ_set:
    mrs r3, cpsr
    cmp r0, #0
    orreq r2, r3, #0x80
    bicne r2, r3, #0x80
    msr cpsr_c, r2
    ands r2, r3, #0x80
    moveq r0, #1
    movne r0, #0
    bx lr

    .global global_FIQ_set
    .type global_FIQ_set, function
# r0==0 disable FIQ
# r0==1 enable FIQ
global_FIQ_set:
    mrs r3, cpsr
    cmp r0, #0
    orreq r2, r3, #0x40
    bicne r2, r3, #0x40
    msr cpsr_c, r2
    ands r2, r3, #0x40
    moveq r0, #1
    movne r0, #0
    bx lr
