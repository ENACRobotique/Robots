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

#define ANGLE_DOOR_OPEN1 180.
#define ANGLE_DOOR_CLOSE1 0.
#define ANGLE_DOOR_OPEN2 180.
#define ANGLE_DOOR_CLOSE2 0.





typedef enum {
    STEP1,
    STEP2
}servoInit;

typedef struct{
    uint8_t club_id;    // identifier of the servomotor (club number)
    uint8_t hw_id;      // plug number (on the card 0-15)
} servoId;

typedef enum {
    DOOR_1,
	DOOR_2
}servoName;


class Servo {
    public:
        Servo(){
            tabServo.insert(std::make_pair(DOOR_1       , servoId{ 2, 8}));
            tabServo.insert(std::make_pair(DOOR_2       , servoId{ 1, 6}));


        }
        ~Servo(){};

        int initServo(){
            std::vector<servoName> name;
            std::vector<float> angle;
            static servoInit step = STEP1;

            switch(step){
                case STEP1:
                    name.push_back(DOOR_1);
                    angle.push_back(ANGLE_DOOR_CLOSE1);

                    name.push_back(DOOR_2);
                    angle.push_back(ANGLE_DOOR_CLOSE2);

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
                    sendPosServo(DOOR_1, ANGLE_DOOR_OPEN1);
                    break;
                case 1 :
                	sendPosServo(DOOR_2, ANGLE_DOOR_OPEN2);
            }
        }

        void closeDoor(int num){
            switch(num){
                case 0 :
                    sendPosServo(DOOR_1, ANGLE_DOOR_CLOSE1);
                    break;
                case 1 :
                	sendPosServo(DOOR_2, ANGLE_DOOR_CLOSE2);
            }
        }
    private:
        std::map<servoName, servoId> tabServo;

};

#endif /* AI_SERVO_H_ */
