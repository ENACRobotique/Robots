/*
 * backlash.c
 *
 *  Created on: 27 mai 2014
 *      Author: ludo6431
 */

#include <assert.h>
#include <stdlib.h>

#include "backlash.h"

void backlash_init(sBackLash *bl, int position_max, int acceleration_hysteresis){
    assert(bl);
    assert(position_max >= 0);
    assert(acceleration_hysteresis >= 0);

    // configuration
    bl->position_max = position_max;
    bl->acceleration_hysteresis = acceleration_hysteresis;

    // initial state (hypotheses: in the middle of the backlash with no speed)
    bl->state = BACKLASH_NO_CONTACT;
    bl->position = position_max>>1;
    bl->prev_motor_incr = 0;
    bl->last_contact_wheel_incr = 0;
}

int backlash_update(sBackLash *bl, int motor_incr /* speed (IpP) */){
    int acceleration, wheel_incr, new_position;

    assert(bl);

    // acceleration estimation
    acceleration = motor_incr - bl->prev_motor_incr; // (IpP²)

    switch(bl->state){
    case BACKLASH_LOW_CONTACT:
        if(acceleration <= bl->acceleration_hysteresis){
            wheel_incr = motor_incr; // contact!
            bl->last_contact_wheel_incr = wheel_incr;
            bl->position = 0; // ensure the backlash position is zero
        }
        else{
            // assert(acceleration > 0)
            // assert(bl->last_contact_wheel_incr == bl->prev_motor_incr)

            new_position = motor_incr - bl->last_contact_wheel_incr + bl->position; // == acceleration + bl->position

            if(new_position > bl->position_max){ // full backlash in one step
                bl->state = BACKLASH_HIGH_CONTACT;
                bl->position = bl->position_max;

                wheel_incr = -bl->position_max + motor_incr; // assume no bouncing
            }
            else{ // fraction of the backlash in one step
                bl->state = BACKLASH_NO_CONTACT;
                bl->position = new_position;

                wheel_incr = bl->last_contact_wheel_incr; // assume no friction
            }
        }
        break;
    case BACKLASH_NO_CONTACT:
        new_position = motor_incr - bl->last_contact_wheel_incr + bl->position;

        if(new_position > bl->position_max){
            bl->state = BACKLASH_HIGH_CONTACT;
            bl->position = bl->position_max;

            wheel_incr = -bl->position_max + motor_incr; // assume no bouncing
        }
        else if(new_position < 0){
            bl->state = BACKLASH_LOW_CONTACT;
            bl->position = 0;

            wheel_incr = bl->position_max + motor_incr; // assume no bouncing
        }
        else{
            bl->position = new_position;

            wheel_incr = bl->last_contact_wheel_incr; // assume no friction
        }

        break;
    case BACKLASH_HIGH_CONTACT:
        if(acceleration >= -bl->acceleration_hysteresis){
            wheel_incr = motor_incr; // contact!
            bl->last_contact_wheel_incr = wheel_incr;
            bl->position = bl->position_max; // ensure the backlash position is maximum
        }
        else{
            // assert(acceleration < 0)
            // assert(bl->last_contact_wheel_incr == bl->prev_motor_incr)

            new_position = motor_incr - bl->last_contact_wheel_incr + bl->position; // == acceleration + bl->position

            if(new_position < 0){ // full backlash in one step
                bl->state = BACKLASH_LOW_CONTACT;
                bl->position = 0;

                wheel_incr = bl->position_max + motor_incr; // assume no bouncing
            }
            else{ // fraction of the backlash in one step
                bl->state = BACKLASH_NO_CONTACT;
                bl->position = new_position;

                wheel_incr = bl->last_contact_wheel_incr; // assume no friction
            }
        }
        break;
    default:
        backlash_init(bl, bl->position_max, bl->acceleration_hysteresis);
        return 0;
    }

    bl->prev_motor_incr = motor_incr;

    return wheel_incr;
}
