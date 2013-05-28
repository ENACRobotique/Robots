/*
 * i2ccomm.h
 *
 *  Created on: 9 mai 2013
 *      Author: quentin
 */

#ifndef I2CCOMM_H_
#define I2CCOMM_H_

#include "stdint.h"

extern uint32_t mesTab[2];

void requestHandler();

typedef enum {
    E_FLOAT_ADDR_M1,
    E_FLOAT_ADDR_M2
}E_TYPEI2C;


typedef struct {
    float dist;
    unsigned long time;
}dtStruct;

typedef struct {
   uint8_t size; // size (in byte) of the comming data to read
   float data;
} tdStruct;


#endif /* I2CCOMM_H_ */
