/*
 * servo.h
 *
 *  Created on: 18 févr. 2015
 *      Author: seb
 */

#ifndef AI_SERVO_H_
#define AI_SERVO_H_

#define MAX_RETRIES_SERVO 1

extern "C"{
#include "messages-interactions.h"
}

typedef struct {
       // eServos id;
        uint16_t u1;      //min servo in millisecond
        uint16_t a1;      //min servo in radian
        uint16_t u2;
        uint16_t a2;
} sServo_t;


class Servo {
    public:
        Servo();
        ~Servo();

       // int sendPosServo(eServos s, float angle /* deg */);

    private:
        sServo_t _servo;
};

#endif /* AI_SERVO_H_ */
