/*
 * data.h
 *
 *  Created on: 18 avr. 2015
 *      Author: quentin
 */

#ifndef SRC_DATA_H_
#define SRC_DATA_H_

#include "shared/lib_synchro_wire.h"

typedef struct {
    uint16_t index;
    uint32_t date;
}wsMeasure_t;

#ifdef ARCH_328P_ARDUINO
extern wsMeasure_t measures[187];
#else
extern wsMeasure_t measures[378];
#endif

#endif /* SRC_DATA_H_ */
