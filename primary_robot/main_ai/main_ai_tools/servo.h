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
#include <vector>
#include <cstdio>
extern "C"{
#include "messages-interactions.h"
}

#define ANGLE_ELEVATOR_INIT1        70.
#define ANGLE_ELEVATOR_LOCK1        90.
#define ANGLE_ELEVATOR_UNLOCK1      0.
#define ANGLE_ELEVATOR_UP1          90.
#define ANGLE_ELEVATOR_DOWN1        0.
#define ANGLE_ELEVATOR_DOWN_ESTRADE1 0.
#define ANGLE_ELEVATOR_OPEN1        90.
#define ANGLE_ELEVATOR_MIDDLE1      45.
#define ANGLE_ELEVATOR_CLOSE1       0.

#define ANGLE_ELEVATOR_LOCK2        128.
#define ANGLE_ELEVATOR_UNLOCK2      90.
#define ANGLE_ELEVATOR_UP2          175.
#define ANGLE_ELEVATOR_DOWN2        3.
#define ANGLE_ELEVATOR_DOWN_ESTRADE2 70.
#define ANGLE_ELEVATOR_DOOR_OPEN2   0.
#define ANGLE_ELEVATOR_DOOR_MIDDLE2 80.
#define ANGLE_ELEVATOR_DOOR_CLOSE2  103.

#define ANGLE_PINCE_IN1             40.  //renter dans le robot
#define ANGLE_PINCE_LOCK1           111. //bloquage verre
#define ANGLE_PINCE_UNLOCK1         152. //ouverture pour prendre le verre
#define ANGLE_PINCE_UP1             150. //renter dans le robot
#define ANGLE_PINCE_INTER1          130. //bouger le robot avec un verre
#define ANGLE_PINCE_DOWN1           62.  //positon basse pour prendre le verre

#define ANGLE_PINCE_IN2             0.
#define ANGLE_PINCE_LOCK2           66.
#define ANGLE_PINCE_UNLOCK2         110.
#define ANGLE_PINCE_UP2             159.
#define ANGLE_PINCE_INTER2          150.
#define ANGLE_PINCE_DOWN2           72.

#define ANGLE_PINCE_IN3             20.
#define ANGLE_PINCE_LOCK3           88.
#define ANGLE_PINCE_UNLOCK3         138.
#define ANGLE_PINCE_UP3             110.
#define ANGLE_PINCE_INTER3          100.
#define ANGLE_PINCE_DOWN3           32.

#define ANGLE_POPCORN_LOADER_OPEN1  90.
#define ANGLE_POPCORN_LOADER_CLOSE1 0.





typedef enum {
    STEP1,
    STEP2,
    STEP3
}servoInit;

typedef struct{
    uint8_t club_id;    // identifier of the servomotor (club number)
    uint8_t hw_id;      // plug number (on the card 0-15)
} servoId;

typedef enum {
    ELEVATOR_UP_DOWN1,
    ELEVATOR_DOOR1,
    ELEVATOR_UNLOCK_LOCK1,
    ELEVATOR_UP_DOWN2,
    ELEVATOR_DOOR2,
    ELEVATOR_UNLOCK_LOCK2,
    PINCE_UP_DOWN1,
    PINCE_UNLOCK_LOCK1,
    PINCE_UP_DOWN2,
    PINCE_UNLOCK_LOCK2,
    PINCE_UP_DOWN3,
    PINCE_UNLOCK_LOCK3,
    POPCORN_LOARDER1
}servoName;


class Servo {
    public:
        Servo(){
            tabServo.insert(std::make_pair(ELEVATOR_UP_DOWN1       , servoId{ 2, 8}));
          /*  tabServo.insert(std::make_pair(ELEVATOR_DOOR1          , servoId{ 0, 0}));
            tabServo.insert(std::make_pair(ELEVATOR_UNLOCK_LOCK1   , servoId{ 0, 0}));*/
            tabServo.insert(std::make_pair(ELEVATOR_UP_DOWN2       , servoId{ 1, 6}));
            tabServo.insert(std::make_pair(ELEVATOR_DOOR2          , servoId{ 9, 0}));
            tabServo.insert(std::make_pair(ELEVATOR_UNLOCK_LOCK2   , servoId{ 7, 4}));
            tabServo.insert(std::make_pair(PINCE_UP_DOWN1          , servoId{12, 11}));
            tabServo.insert(std::make_pair(PINCE_UNLOCK_LOCK1      , servoId{13, 15}));
            tabServo.insert(std::make_pair(PINCE_UP_DOWN2          , servoId{16, 13}));
            tabServo.insert(std::make_pair(PINCE_UNLOCK_LOCK2      , servoId{15, 14}));
            tabServo.insert(std::make_pair(PINCE_UP_DOWN3          , servoId{18, 5}));
            tabServo.insert(std::make_pair(PINCE_UNLOCK_LOCK3      , servoId{17, 7}));


        }
        ~Servo(){};

        int initServo(){
            std::vector<servoName> name;
            std::vector<float> angle;
            static servoInit step = STEP1;

            switch(step){
                case STEP1:
                    name.push_back(PINCE_UNLOCK_LOCK1);
                    angle.push_back(ANGLE_PINCE_IN1);

                    name.push_back(PINCE_UNLOCK_LOCK2);
                    angle.push_back(ANGLE_PINCE_IN2);

                    name.push_back(PINCE_UNLOCK_LOCK3);
                    angle.push_back(ANGLE_PINCE_IN3);

                    name.push_back(ELEVATOR_UP_DOWN2);
                    angle.push_back(ANGLE_ELEVATOR_UP2);

                    name.push_back(ELEVATOR_DOOR2);
                    angle.push_back(ANGLE_ELEVATOR_DOOR_CLOSE2);

                    name.push_back(ELEVATOR_UNLOCK_LOCK2);
                    angle.push_back(ANGLE_ELEVATOR_LOCK2);

                    sendMultiPosServo(name, angle);
                    step = STEP2;
                    break;
                case STEP2:
                    name.push_back(PINCE_UP_DOWN1);
                    angle.push_back(ANGLE_PINCE_UP1);

                    name.push_back(PINCE_UP_DOWN2);
                    angle.push_back(ANGLE_PINCE_UP2);

                    name.push_back(PINCE_UP_DOWN3);
                    angle.push_back(ANGLE_PINCE_UP3);

                    name.push_back(ELEVATOR_UP_DOWN1);
                    angle.push_back(ANGLE_ELEVATOR_INIT1);

                    sendMultiPosServo(name, angle);
                    step = STEP3;
                    break;
                case STEP3:

                    return 1;
                    break;

            }

            return 0;
        }

        int sendPosServo(const servoName name, const float angle /* deg */);
        int sendMultiPosServo(const std::vector<servoName> name, const std::vector<float> angle);

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
        void downEstradeElevator(int num){
            sendPosServo(num==0?ELEVATOR_UP_DOWN1:ELEVATOR_UP_DOWN2,
                    num==0?ANGLE_ELEVATOR_DOWN_ESTRADE1:ANGLE_ELEVATOR_DOWN_ESTRADE2);
        }
        void openDoorElevator(int num){
            switch(num){
                case 0:
                    sendPosServo(ELEVATOR_DOOR1, ANGLE_ELEVATOR_OPEN1);
                    break;
                case 1:
                    sendPosServo(ELEVATOR_DOOR2, ANGLE_ELEVATOR_DOOR_OPEN2);
                    break;
            }
        }
        void middleDoorElevator(int num){
            switch(num){
                case 0:
                    sendPosServo(ELEVATOR_DOOR1, ANGLE_ELEVATOR_MIDDLE1);
                    break;
                case 1:
                    sendPosServo(ELEVATOR_DOOR2, ANGLE_ELEVATOR_DOOR_MIDDLE2);
                    break;
            }
        }
        void closeDoorElevator(int num){
            switch(num){
                case 0:
                    sendPosServo(ELEVATOR_DOOR1, ANGLE_ELEVATOR_CLOSE1);
                    break;
                case 1:
                    sendPosServo(ELEVATOR_DOOR2, ANGLE_ELEVATOR_DOOR_CLOSE2);
                    break;
            }
        }
        void inPince(int num){
            switch(num){
                case 0 :
                    sendPosServo(PINCE_UNLOCK_LOCK1, ANGLE_PINCE_IN1);
                    break;
                case 1 :
                    sendPosServo(PINCE_UNLOCK_LOCK2, ANGLE_PINCE_IN2);
                    break;
                case 2 :
                    sendPosServo(PINCE_UNLOCK_LOCK3, ANGLE_PINCE_IN3);
                    break;
            }
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
        void interPince(int num){
            printf("\ninterPince.........:%d\n", num);
            switch(num){
                case 0 :
                    sendPosServo(PINCE_UP_DOWN1, ANGLE_PINCE_INTER1);
                    break;
                case 1 :
                    sendPosServo(PINCE_UP_DOWN2, ANGLE_PINCE_INTER2);
                    break;
                case 2 :
                    sendPosServo(PINCE_UP_DOWN3, ANGLE_PINCE_INTER3);
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
