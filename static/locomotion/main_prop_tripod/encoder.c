#include <ime.h>

#include <encoder.h>

void encoder_init(encoder_t* e, encoder_polarity_t pol, eEINT eint, eEINT_PINASSIGN eint_pin, eEINT_MODE eint_type, eint_handler eint_h, int eint_prio) {
    e->eint = eint;
    e->pol = pol;

    eint_disable(eint);
    eint_assign(eint_pin);
    eint_mode(eint, eint_type);
    eint_register(eint, eint_h, eint_prio);
    eint_enable(eint);
}

int encoder_read(encoder_t* e) {
    int nbticks;

    global_IRQ_disable();
    nbticks = e->nbticks;
    e->nbticks = 0;
    global_IRQ_enable();

    return e->pol == POSITIVE_IS_TRIGO ? nbticks : -nbticks;
}

void encoder_deinit(encoder_t* e) {
    eint_disable(e->eint);
}

int get_encoder(encoder_t* e){
    return e->nbticks;
}
