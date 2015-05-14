/*
 * servoTools.cpp
 *
 *  Created on: 14 mai 2015
 *      Author: seb
 */

#include "servoTools.h"

#include "tools.h"

extern "C" {
#include "millis.h"
}

typedef enum {
    GET_STAND_ENTRY,
    GET_STAND_OPEN,
    GET_STAND_DOWN,
    GET_STAND_LOCK,
    GET_STAND_UP
}stepGetStand;


int getStand(unsigned int id){
    static stepGetStand step = GET_STAND_ENTRY;
    static unsigned int timePrev = 0;

    switch(step){
        case GET_STAND_ENTRY:
            //TODO detection new stand
            step = GET_STAND_OPEN;
            break;

        case GET_STAND_OPEN:
            servo.unlockElevator(id);
            servo.middleDoorElevator(id);
            timePrev = millis();
            step = GET_STAND_DOWN;
            break;

        case GET_STAND_DOWN:
            if(millis() - timePrev > 500){
                servo.downElevator(id);
                timePrev = millis();
                step = GET_STAND_LOCK;
            }
            break;

        case GET_STAND_LOCK:
            if(millis() - timePrev > 500){
                servo.lockElevator(id);
                servo.closeDoorElevator(id);
                timePrev = millis();
                step = GET_STAND_UP;
            }
            break;

        case GET_STAND_UP:
            if(millis() - timePrev > 500){
                servo.upElevator(id);
                timePrev = 0;
                step = GET_STAND_ENTRY;
                return 1;
            }
            break;

    }

    return 0;
}

