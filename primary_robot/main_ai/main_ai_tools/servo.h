/*
 * servo.h
 *
 *  Created on: 18 févr. 2015
 *      Author: seb
 */

#ifndef AI_SERVO_H_
#define AI_SERVO_H_

#define MAX_RETRIES_SERVO 5

#include <map>
extern "C"{
#include "messages-interactions.h"
}

typedef enum {
    ELEVATOR_LOW_TOP1,
    ELEVATOR_LOCK1,
    ELEVATOR_LOW_TOP2,
    ELEVATOR_LOCK2,
}servoName;

typedef struct{
    uint8_t club_id;    // identifier of the servomotor (club number)
    uint8_t hw_id;      // plug number (on the card 0-15)
} servoId;




class Servo {
    public:
        Servo(){
            tabServo.insert(std::make_pair(ELEVATOR_LOW_TOP1, servoId{0, 0}));
            tabServo.insert(std::make_pair(ELEVATOR_LOCK1   , servoId{0, 0}));
            tabServo.insert(std::make_pair(ELEVATOR_LOW_TOP1, servoId{0, 0}));
            tabServo.insert(std::make_pair(ELEVATOR_LOCK2   , servoId{0, 0}));
        }
        ~Servo(){};

        int sendPosServo(const servoName name, const float angle /* rad */);

    private:
        std::map<servoName, servoId> tabServo;

};

#endif /* AI_SERVO_H_ */
