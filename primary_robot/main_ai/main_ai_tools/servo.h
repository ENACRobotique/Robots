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
extern "C"{
#include "messages-interactions.h"
}

#define ANGLE_DOOR_OPEN1L 36.
#define ANGLE_DOOR_CLOSE1L 150.
#define ANGLE_DOOR_OPEN1R 175.
#define ANGLE_DOOR_CLOSE1R 85.





typedef enum {
    STEP1,
    STEP2
}servoInit;

typedef struct{
    uint8_t club_id;    // identifier of the servomotor (club number)
    uint8_t hw_id;      // plug number (on the card 0-15)
} servoId;

typedef enum {
    DOOR_1_L,
	DOOR_1_R
}servoName;


class Servo {
    public:
        Servo(){
            tabServo.insert(std::make_pair(DOOR_1_L      , servoId{ 12, 1}));
            tabServo.insert(std::make_pair(DOOR_1_R       , servoId{ 2, 2}));


        }
        ~Servo(){};

        int initServo(){
            std::vector<servoName> name;
            std::vector<float> angle;
            static servoInit step = STEP1;

            switch(step){
                case STEP1:
                    name.push_back(DOOR_1_L);
                    angle.push_back(ANGLE_DOOR_CLOSE1L);

                    name.push_back(DOOR_1_R);
                    angle.push_back(ANGLE_DOOR_CLOSE1R);

                    sendMultiPosServo(name, angle);
                    step = STEP2;
                    break;
                case STEP2:

                    return 1;
                    break;

            }

            return 0;
        }

        int sendPosServo(const servoName name, const float angle /* deg */);
        int sendMultiPosServo(const std::vector<servoName> name, const std::vector<float> angle);

        void openDoor(int num){
            switch(num){
                case 0 :
                    sendPosServo(DOOR_1_L, ANGLE_DOOR_OPEN1L);
                    break;
                case 1 :
                	sendPosServo(DOOR_1_R, ANGLE_DOOR_OPEN1R);
            }
        }

        void closeDoor(int num){
            switch(num){
                case 0 :
                    sendPosServo(DOOR_1_L, ANGLE_DOOR_CLOSE1L);
                    break;
                case 1 :
                	sendPosServo(DOOR_1_R, ANGLE_DOOR_CLOSE1R);
            }
        }
    private:
        std::map<servoName, servoId> tabServo;

};

#endif /* AI_SERVO_H_ */
