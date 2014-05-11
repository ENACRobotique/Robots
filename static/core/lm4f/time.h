/*
 * time.h
 *
 *  Created on: 11 mai 2014
 *      Author: quentin
 */

#ifndef TIME_H_
#define TIME_H_

#include "stdint.h"

void SysTickIntHandler(void);
void Timer5IntHandler(void);
void timerInit();
unsigned long micros(void);
unsigned long millis(void);
void delayMicroseconds(unsigned int us);
void delay(uint32_t milliseconds);
void registerSysTickCb(void (*userFunc)(uint32_t));


#endif /* TIME_H_ */
