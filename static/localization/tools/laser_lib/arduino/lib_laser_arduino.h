/*
 * lib_laser_arduino.h
 *
 *  Created on: 18 mai 2014
 *      Author: quentin
 */

#if !(definedLIB_LASER_ARDUINO_H_) && defined(ARCH_328P_ARDUINO)
#define LIB_LASER_ARDUINO_H_

#include "../shared/lib_int_laser.h"



//void laser_arduino_IntHand0();
//void laser_arduino_IntHand1();
void laser_arduino_Intinit();
void laser_arduino_Intdeinit();
#endif /* LIB_LASER_ARDUINO_H_ */
