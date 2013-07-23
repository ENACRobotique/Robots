/*
 * lib_domitille.h
 *
 *  Created on: 4 mai 2013
 *      Author: quentin
 */

#ifndef LIB_DOMITILLE_H_
#define LIB_DOMITILLE_H_

extern volatile unsigned long TR_period, TR_mean_period;
extern volatile unsigned long last_TR;

//prototypes
void domi_isr();
void domi_init(int pinInt);
void domi_deinit();

inline unsigned long domi_period(){
    return TR_period;
}

inline unsigned long domi_meanPeriod(){
    return TR_mean_period;
}

#endif /* LIB_DOMITILLE_H_ */
