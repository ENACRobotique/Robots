#ifndef MOT_ENC_H
#include "params.h"
//#ifdef ARCH_X86_LINUX // FIXME
#include <math.h>
#include "sys_time.h"
#include "motor.h"
#include "encoder.h"

int mot_enc_getticks(motor_t *m, encoder_t *enc);

//#endif // ARCH_X86_LINUX
#endif // MOT_END_H
