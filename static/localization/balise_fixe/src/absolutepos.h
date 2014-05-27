/*
 * absolutepos.h
 *
 *  Created on: 25 mai 2014
 *      Author: ThomasDq
 */

#ifndef ABSOLUTEPOS_H_
#define ABSOLUTEPOS_H_

#include "time.h"
#include "neldermead.h"
#include "stdint.h"
#include "perception.h"
#include "../../communication/botNet/shared/botNet_core.h"
#include "../../communication/network_tools/bn_debug.h"
#include "roles.h"
#include "inc/lm4f120h5qr.h"
#include <driverlib/fpu.h>
#include "tools.h"


#include <stdlib.h>

#ifndef BIT
#define BIT(a) (1<<a)
#endif
#define RANGE 0.05


void absolutepos(sMeasures*buffer,int index, int taille);

#endif /* ABSOLUTEPOS_H_ */
