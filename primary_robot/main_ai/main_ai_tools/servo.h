/*
 * servo.h
 *
 *  Created on: 18 févr. 2015
 *      Author: seb
 */

#ifndef AI_SERVO_H_
#define AI_SERVO_H_

#define MAX_RETRIES_SERVO 2

#include <map>
extern "C"{
#include "messages-interactions.h"
}

#define ANGLE_ELEVATOR_LOCK1        90.
#define ANGLE_ELEVATOR_LOCK2        90.
#define ANGLE_ELEVATOR_UNLOCK1      0.
#define ANGLE_ELEVATOR_UNLOCK2      0.
#define ANGLE_ELEVATOR_UP1          90.
#define ANGLE_ELEVATOR_UP2          90.
#define ANGLE_ELEVATOR_DOWN1        0.
#define ANGLE_ELEVATOR_DOWN2        0.

#define ANGLE_PINCE_LOCK1           90.
#define ANGLE_PINCE_UNLOCK1         0.
#define ANGLE_PINCE_UP1             90.
#define ANGLE_PINCE_DOWN1           0.

#define ANGLE_PINCE_LOCK2           90.
#define ANGLE_PINCE_UNLOCK2         0.
#define ANGLE_PINCE_UP2             90.
#define ANGLE_PINCE_DOWN2           0.

#define ANGLE_PINCE_LOCK3           90.
#define ANGLE_PINCE_UNLOCK3         0.
#define ANGLE_PINCE_UP3             90.
#define ANGLE_PINCE_DOWN3           0.

#define ANGLE_POPCORN_LOADER_OPEN1  90.
#define ANGLE_POPCORN_LOADER_CLOSE1 0.



typedef enum {
    ELEVATOR_UP_DOWN1,
    ELEVATOR_UNLOCK_LOCK1,
    ELEVATOR_UP_DOWN2,
    ELEVATOR_UNLOCK_LOCK2,
    PINCE_UP_DOWN1,
    PINCE_UNLOCK_LOCK1,
    PINCE_UP_DOWN2,
    PINCE_UNLOCK_LOCK2,
    PINCE_UP_DOWN3,
    PINCE_UNLOCK_LOCK3,
    POPCORN_LOARDER1
}servoName;

typedef struct{
    uint8_t club_id;    // identifier of the servomotor (club number)
    uint8_t hw_id;      // plug number (on the card 0-15)
} servoId;




class Servo {
    public:
        Servo(){
            tabServo.insert(std::make_pair(ELEVATOR_UP_DOWN1, servoId{0, 0}));
            tabServo.insert(std::make_pair(ELEVATOR_UNLOCK_LOCK1   , servoId{0, 0}));
            tabServo.insert(std::make_pair(ELEVATOR_UP_DOWN1, servoId{0, 0}));
            tabServo.insert(std::make_pair(ELEVATOR_UNLOCK_LOCK2   , servoId{0, 0}));
        }
        ~Servo(){};

        int sendPosServo(const servoName name, const float angle /* deg */);

        void lockElevator(int num){
            sendPosServo(num==0?ELEVATOR_UNLOCK_LOCK1:ELEVATOR_UNLOCK_LOCK2,
                    num==0?ANGLE_ELEVATOR_LOCK1:ANGLE_ELEVATOR_LOCK2);
        }
        void unlockElevator(int num){
            sendPosServo(num==0?ELEVATOR_UNLOCK_LOCK1:ELEVATOR_UNLOCK_LOCK2,
                    num==0?ANGLE_ELEVATOR_UNLOCK1:ANGLE_ELEVATOR_UNLOCK2);
        }
        void upElevator(int num){
            sendPosServo(num==0?ELEVATOR_UP_DOWN1:ELEVATOR_UP_DOWN2,
                    num==0?ANGLE_ELEVATOR_UP1:ANGLE_ELEVATOR_UP2);
        }
        void downElevator(int num){
            sendPosServo(num==0?ELEVATOR_UP_DOWN1:ELEVATOR_UP_DOWN2,
                    num==0?ANGLE_ELEVATOR_DOWN1:ANGLE_ELEVATOR_DOWN2);
        }
        void lockPince(int num){
            switch(num){
                case 0 :
                    sendPosServo(PINCE_UNLOCK_LOCK1, ANGLE_PINCE_LOCK1);
                    break;
                case 1 :
                    sendPosServo(PINCE_UNLOCK_LOCK2, ANGLE_PINCE_LOCK2);
                    break;
                case 2 :
                    sendPosServo(PINCE_UNLOCK_LOCK3, ANGLE_PINCE_LOCK3);
                    break;
            }

        }
        void unlockPince(int num){
            switch(num){
                case 0 :
                    sendPosServo(PINCE_UNLOCK_LOCK1, ANGLE_PINCE_UNLOCK1);
                    break;
                case 1 :
                    sendPosServo(PINCE_UNLOCK_LOCK2, ANGLE_PINCE_UNLOCK2);
                    break;
                case 2 :
                    sendPosServo(PINCE_UNLOCK_LOCK3, ANGLE_PINCE_UNLOCK3);
                    break;
            }

        }
        void upPince(int num){
            switch(num){
                case 0 :
                    sendPosServo(PINCE_UP_DOWN1, ANGLE_PINCE_UP1);
                    break;
                case 1 :
                    sendPosServo(PINCE_UP_DOWN2, ANGLE_PINCE_UP2);
                    break;
                case 2 :
                    sendPosServo(PINCE_UP_DOWN3, ANGLE_PINCE_UP3);
                    break;
            }

        }
        void downPince(int num){
            switch(num){
                case 0 :
                    sendPosServo(PINCE_UP_DOWN1, ANGLE_PINCE_DOWN1);
                    break;
                case 1 :
                    sendPosServo(PINCE_UP_DOWN2, ANGLE_PINCE_DOWN2);
                    break;
                case 2 :
                    sendPosServo(PINCE_UP_DOWN3, ANGLE_PINCE_DOWN3);
                    break;
            }
        }
        void openPopcornLoader(int num){
            switch(num){
                case 0 :
                    sendPosServo(POPCORN_LOARDER1, ANGLE_POPCORN_LOADER_OPEN1);
                    break;
            }
        }
        void closePopcornLoader(int num){
            switch(num){
                case 0 :
                    sendPosServo(POPCORN_LOARDER1, ANGLE_POPCORN_LOADER_CLOSE1);
                    break;
            }
        }
    private:
        std::map<servoName, servoId> tabServo;

};

#endif /* AI_SERVO_H_ */
