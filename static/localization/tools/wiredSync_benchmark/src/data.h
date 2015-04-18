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
    int index;
    uint32_t date;
    uint32_t unoffset_date;
}wsMeasure_t;

extern wsMeasure_t measures[378];

#endif /* SRC_DATA_H_ */
