/*
 * backlash.h
 *
 *  Created on: 27 mai 2014
 *      Author: ludo6431
 */

#ifndef BACKLASH_H_
#define BACKLASH_H_

#include <stdint.h>

typedef enum{
    BACKLASH_LOW_CONTACT,
    BACKLASH_NO_CONTACT,
    BACKLASH_HIGH_CONTACT
} eBacklashState;

typedef struct{
    int position_max;
    int acceleration_hysteresis;

    eBacklashState state;
    int position; // from 0 to position_max (0:LOW_CONTACT | position_max:HIGH_CONTACT | *:NO_CONTACT)
    int prev_motor_incr; // homogeneous to a speed (IpP : Increments per Sampling Period)
    int last_contact_wheel_incr; // homogeneous to a speed
} sBackLash;

void backlash_init(sBackLash *bl, int amplitude, int hysteresis);
int backlash_update(sBackLash *bl, int motor_incr /* speed (IpP) */);

#endif /* BACKLASH_H_ */
