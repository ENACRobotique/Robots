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

typedef enum {
    DROP_CUP_INIT,
    DROP_CUP_DOWN,
    DROP_CUP_UNLOCK,
    DROP_CUP_END
}stepDropCup;

typedef enum {
    CUP_CLOSE,
    CUP_UP,
    CUP_END
}stepRangePince;

typedef enum {
    DROP_STAND_DOWN_ELEVATOR,
    DROP_STAND_UNLOCK,
    DROP_STAND_OPEN_DOOR,
    DROP_STAND_END,
}stepdropStand;


int getStand(unsigned int id){
    static stepGetStand step = GET_STAND_ENTRY;
    static unsigned int timePrev = 0;

    switch(step){
        case GET_STAND_ENTRY:
            //TODO detection new stand (ya pas!!!)
            step = GET_STAND_OPEN;
            break;

        case GET_STAND_OPEN:
            servo.unlockElevator(id);
            servo.middleDoorElevator(id);
            timePrev = millis();
            step = GET_STAND_DOWN;
            break;

        case GET_STAND_DOWN:
            if(millis() - timePrev > 1000){
                servo.downElevator(id);
                timePrev = millis();
                step = GET_STAND_LOCK;
            }
            break;

        case GET_STAND_LOCK:
            if(millis() - timePrev > 1000){
                servo.lockElevator(id);
                servo.closeDoorElevator(id);
                timePrev = millis();
                step = GET_STAND_UP;
            }
            break;

        case GET_STAND_UP:
            if(millis() - timePrev > 1500){
                servo.upElevator(id);
                timePrev = 0;
                step = GET_STAND_ENTRY;
                return 1;
            }
            break;

    }
    return 0;
}

int dropCup(unsigned int id){
    static stepDropCup step = DROP_CUP_INIT;
    static unsigned int timePrev = 0;

    switch(step){
        case DROP_CUP_INIT:
            timePrev = millis();
            step = DROP_CUP_DOWN;
            break;

        case DROP_CUP_DOWN:
            if(millis() - timePrev > 1000){
                servo.downPince(id);
                timePrev = millis();
                //step = DROP_CUP_UNLOCK;
                step = DROP_CUP_END;
            }
            break;

        case DROP_CUP_UNLOCK:
            if(millis() - timePrev > 1000){
                servo.unlockPince(id);
                timePrev = millis();
                step = DROP_CUP_END;
            }
            break;

        case DROP_CUP_END:
            if(millis() - timePrev > 1000){
                step = DROP_CUP_INIT;
                timePrev = 0;
                return 1;
            }
            break;
    }

    return 0;
}

int rangePince(unsigned int id){
    static stepRangePince step = CUP_CLOSE;
    static unsigned int timePrev = 0;

    switch(step){
        case CUP_CLOSE:
            servo.lockPince(id);
            timePrev = millis();
            step = CUP_UP;
            break;

        case CUP_UP:
            if(millis() - timePrev > 500){
                servo.upPince(id);
                timePrev = millis();
                step = CUP_END;
            }
            break;

        case CUP_END:
            if(millis() - timePrev > 500){
                step = CUP_CLOSE;
                timePrev = 0;
                return 1;
            }
            break;
    }
    return 0;
}


int dropStand(unsigned int id){
    static stepdropStand step = DROP_STAND_DOWN_ELEVATOR;
    static unsigned int timePrev = 0;

    switch(step){
        case DROP_STAND_DOWN_ELEVATOR:
            //servo.downElevator(id);
            servo.downEstradeElevator(id);
            timePrev = millis();
            step = DROP_STAND_UNLOCK;
            break;

        case DROP_STAND_UNLOCK:
            if(millis() - timePrev > 1000){
                servo.unlockElevator(id);
                timePrev = millis();
                step = DROP_STAND_OPEN_DOOR;
            }
            break;

        case DROP_STAND_OPEN_DOOR:
            if(millis() - timePrev > 500){
                servo.openDoorElevator(id);
                timePrev = millis();
                step = DROP_STAND_END;
            }
            break;

        case DROP_STAND_END:
            if(millis() - timePrev > 1000){
                step = DROP_STAND_DOWN_ELEVATOR;
                timePrev = 0;
                return 1;
            }
            break;
    }
    return 0;
}

