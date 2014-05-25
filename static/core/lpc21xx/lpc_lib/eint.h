#ifndef _EINT_H
#define _EINT_H

#include <lpc214x.h>

#ifndef BIT
#define BIT(b) (1<<(b))
#endif

/* Typical init:
 *      eint_disable(EINT0);
 *      eint_assign(EINT0_P0_1);
 *      eint_mode(EINT0, EINT_RISING_EDGE);
 *      eint_register(EINT0, some_func, 2);
 *      eint_enable(EINT0);
 */

typedef enum{
    EINT0,
    EINT1,
    EINT2,
    EINT3
} eEINT;

typedef enum{
    EINT0_P0_1,
    EINT0_P0_16,
    EINT1_P0_3,
    EINT1_P0_14,
    EINT2_P0_7,
    EINT2_P0_15,
    EINT3_P0_9,
    EINT3_P0_20,
    EINT3_P0_30
} eEINT_PINASSIGN;

typedef enum{ // order and values are meaningful
    EINT_LOW_LEVEL,
    EINT_HIGH_LEVEL,
    EINT_FALLING_EDGE,
    EINT_RISING_EDGE
} eEINT_MODE;

static inline void eint_disable(eEINT i){
    VIC_IntEnClr = BIT(14 + i); // disable interrupt
}

static inline void eint_enable(eEINT i){
    VIC_IntSelect &= ~BIT(14 + i); // IRQ (not FIQ)
    // FIXME classify those 2 interrupts as FIQ using VICIntSelect

    SCB_EXTINT = BIT(i); // clear interrupt flag
    VIC_IntEnable = BIT(14 + i); // enable interrupt
}

static inline void eint_assign(eEINT_PINASSIGN p){
    switch(p){
    case EINT0_P0_1:
        PCB_PINSEL0 |= 3<<2;
        break;
    case EINT0_P0_16:
        PCB_PINSEL1 = (PCB_PINSEL1 & ~(3<<0)) | (1<<0);
        break;
    case EINT1_P0_3:
        PCB_PINSEL0 |= 3<<6;
        break;
    case EINT1_P0_14:
        PCB_PINSEL0 = (PCB_PINSEL0 & ~(3<<28)) | (2<<28);
        break;
    case EINT2_P0_7:
        PCB_PINSEL0 |= 3<<14;
        break;
    case EINT2_P0_15:
        PCB_PINSEL0 = (PCB_PINSEL0 & ~(3<<30)) | (2<<30);
        break;
    case EINT3_P0_9:
        PCB_PINSEL0 |= 3<<18;
        break;
    case EINT3_P0_20:
        PCB_PINSEL1 |= 3<<8;
        break;
    case EINT3_P0_30:
        PCB_PINSEL1 = (PCB_PINSEL1 & ~(3<<28)) | (2<<28);
        break;
    default:
        break;
    }
}

static inline void eint_mode(eEINT i, eEINT_MODE m){
    if(m&BIT(0)){
        SCB_EXTPOLAR |= BIT(i);
    }
    else{
        SCB_EXTPOLAR &= ~BIT(i);
    }

    if(m&BIT(1)){
        SCB_EXTMODE |= BIT(i);
    }
    else{
        SCB_EXTMODE &= ~BIT(i);
    }
}

typedef void (*eint_handler)();

static inline eint_handler eint_register(eEINT i, eint_handler h, int priority){
    eint_handler prev_h = (eint_handler)VIC_VectAddr2;

    VIC_VectCntlN(priority) = BIT(5) | (14 + i);
    VIC_VectAddrN(priority) = (unsigned)h;

    return prev_h;
}

#endif
