/*
 * servo.cpp
 *
 *  Created on: 18 févr. 2015
 *      Author: seb
 */

#include "servo.h"

#include <cmath>
extern "C"{
#include <string.h>
#include "network_cfg.h"
#include "botNet_core.h"
}

int Servo::sendPosServo(const servoName name, const float angle) {
    sMsg msg;

    memset(&msg, 0, sizeof(msg));

    if(angle < 0)
        return -1;

    msg.header.destAddr = ADDRI_MAIN_IO;
    msg.header.type = E_SERVOS;
    msg.payload.servos.nb_servos = 1;
    msg.header.size = 2 + 6*msg.payload.servos.nb_servos;
    msg.payload.servos.servos[0].club_id = tabServo[name].club_id;
    msg.payload.servos.servos[0].hw_id = tabServo[name].hw_id;
    msg.payload.servos.servos[0].angle = angle;

    bn_sendRetry(&msg, MAX_RETRIES_SERVO);
    return 1;
}

