#include "mot_enc.h"

//#ifdef ARCH_X86_LINUX // FIXME
int mot_enc_getticks(motor_t *m, encoder_t *enc){
    unsigned int time = millis();

    // Low pass filter pattern of the motor
    m->speed = (int) (m->setPoint + (float)(m->speed - m->setPoint)*expf(-(float) (time - enc->lastticksquery)/MOT_TIME_CST));
    enc->lastticksquery = time;

    return m->speed;
}
//#endif
