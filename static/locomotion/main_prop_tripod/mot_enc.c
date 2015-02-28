#include "mot_enc.h"


int mot_enc_getticks(motor_t *m, encoder_t *enc){
    unsigned int time = millis();
    static unsigned int lastticksquery;

    // Low pass filter pattern of the motor
    m->speed = (int) (m->setPoint + (float)(m->speed - m->setPoint)*expf(-(float) (time - m->lastticksquery)/MOT_TIME_CST));
    lastticksquery = time;

    return m->speed;
}
