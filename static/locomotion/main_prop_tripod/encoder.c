#include "encoder.h"

void encoder_init(encoder_t* e, eEINT eint, eEINT_PINASSIGN eint_pin, eEINT_MODE eint_mode, eint_handler eint_h, int eint_prio){
    e->eint = eint;

    eint_disable(eint);
    eint_assign(eint_pin);
    eint_mode(eint, eint_mode);
    eint_register(eint, eint_h, eint_prio);
    eint_enable(eint);
}

int encoder_read(encoder_t* e){
    int nbticks;

    global_IRQ_disable();
    nbticks = e->nbticks;
    e->nbticks = 0;
    global_IRQ_enable();

    return nbticks;
}

void encoder_deinit(encoder_t* e){
    eint_disable(e->eint);
}
