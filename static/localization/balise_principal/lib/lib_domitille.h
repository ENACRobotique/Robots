/*
 * lib_domitille.h
 *
 *  Created on: 4 mai 2013
 *      Author: quentin
 */

#ifndef LIB_DOMITILLE_H_
#define LIB_DOMITILLE_H_

#include "stdint.h"

#define FILTER_SHIFT 2

#define TR_INFO_BUFFER_SIZE 16

#define SPEED_HIGH  200
#define SPEED_20HZ  46

typedef struct{
    uint32_t date;
    uint32_t period;
}sTurnInfo;


extern volatile unsigned int _nbTR;
extern volatile uint32_t TR_mean_period,TR_lastDate;

// Rotating buffer recording informations about few last turns (to compute angle)
extern volatile int TR_iNext;   // index for the rotating buffer of the last period (current turn, for TR_infobuf[TR_cur], ONLY the date is set, not the period )
extern volatile sTurnInfo TR_InfoBuf[TR_INFO_BUFFER_SIZE];


//prototypes
void domi_isr();
void domi_init(int pinInt, int pinSpeed);
void domi_deinit();
void domi_setspeed(int speed);

inline int domi_iCur(){
    return (TR_iNext-1+TR_INFO_BUFFER_SIZE)%TR_INFO_BUFFER_SIZE;
}

inline int domi_nbTR(){
    return _nbTR;
}

inline void domi_resetNbTR(){
    _nbTR=0;
}
inline unsigned long domi_lastTR(){
    return TR_InfoBuf[domi_iCur()].date;
}

inline unsigned long domi_lastPeriod(){
    return TR_InfoBuf[domi_iCur()].period;
}

inline unsigned long domi_meanPeriod(){
    return TR_mean_period;
}

#endif /* LIB_DOMITILLE_H_ */
