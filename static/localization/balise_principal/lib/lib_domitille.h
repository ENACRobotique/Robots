/*
 * lib_domitille.h
 *
 *  Created on: 4 mai 2013
 *      Author: quentin
 */

#ifndef LIB_DOMITILLE_H_
#define LIB_DOMITILLE_H_

extern volatile unsigned int _nbTR;
extern volatile uint32_t last_TR,TR_period,TR_mean_period;

//prototypes
void domi_isr();
void domi_init(int pinInt);
void domi_deinit();

inline int domi_nbTR(){
    return _nbTR;
}
inline void domi_resetNbTR(){
    _nbTR=0;
}
inline unsigned long domi_lastTR(){
    return last_TR;
}

inline unsigned long domi_period(){
    return TR_period;
}

inline unsigned long domi_meanPeriod(){
    return TR_mean_period;
}

#endif /* LIB_DOMITILLE_H_ */
