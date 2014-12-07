/*
 * absolutepos.h
 *
 *  Created on: 25 mai 2014
 *      Author: ThomasDq
 */

#ifndef ABSOLUTEPOS_H_
#define ABSOLUTEPOS_H_

#include <perception.h>

#ifndef BIT
#define BIT(a) (1<<a)
#endif
#define RANGE 0.05


void absolutepos(sMeasures*buffer,int index, int taille);

#endif /* ABSOLUTEPOS_H_ */
