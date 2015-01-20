#include "encoder.h"

volatile int irqCpt = 0;
volatile int dir_mes = 0; // clockwise rotation = -1; trigonometry = 1

// EINT0 sur P0,1 => Channel A
void isr_eint0() __attribute__ ((interrupt("IRQ")));
void isr_eint0() {
    SCB_EXTINT = BIT(0); // acknowledges interrupt
    VIC_VectAddr = (unsigned) 0; // updates priority hardware

    if (ChannelB) // B =1
        dir_mes = DIR_MES_TRIGO; // sens = +1
    else
        dir_mes = DIR_MES_HORAI; // sens = -1

    irqCpt = irqCpt + dir_mes;
}

// EINT3 sur P0,20 => Channel B
void isr_eint3() __attribute__ ((interrupt("IRQ")));
void isr_eint3() {
    SCB_EXTINT = BIT(3); // acknowledges interrupt
    VIC_VectAddr = (unsigned) 0; // updates priority hardware

    if (ChannelA)
        dir_mes = DIR_MES_HORAI; // sens = -1
    else
        dir_mes = DIR_MES_TRIGO; // sens = +1

    irqCpt = irqCpt + dir_mes;
}
