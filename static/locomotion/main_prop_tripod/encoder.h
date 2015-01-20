#ifndef ENCODER_H
#define ENCODER_H

#include <lpc214x.h>
#include "param.h"
#include <eint.h>
#include <ime.h>
#include <gpio.h>

#define DIR_MES_TRIGO 1
#define DIR_MES_HORAI -1
#define NB_CHANNEL 2
#define RES_CHANNEL 500 // inc/turn

extern volatile int irqCpt;
extern volatile int irqCptRef;
extern volatile int dir_mes;

void isr_eint0();
void isr_eint3();

#endif
