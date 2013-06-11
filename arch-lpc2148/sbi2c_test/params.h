#ifndef _PARAMS_H
#define _PARAMS_H

#include "network_cfg.h"

#define MYADDRI ADDRI_MAIN_CANDLE
#define MYADDRX 0
#define SB_INC_MSG_BUF_SIZE 4
#define ARCH_LPC21XX

#define SHIFT (8)
#define dSHIFT ((double)(1<<SHIFT))
#define iROUND(d) ((int)( (d)+0.5 )) // the +0.5 is here to get a round instead of a floor when casting to int
#define isROUND(d) iROUND((d)*dSHIFT)
#define lROUND(d) ((long long)( (d)+0.5 )) // the +0.5 is here to get a round instead of a floor when casting to int
#define lsROUND(d) lROUND((d)*dSHIFT)

#define PI (3.141592654)
#define sPI (PI*dSHIFT)
#define isPI (iROUND(sPI))  // (rad<<SHIFT)

#endif
