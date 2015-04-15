/*
 * servo.cpp
 *
 *  Created on: 18 févr. 2015
 *      Author: seb
 */

#include <main_ai_tools/servo.h>
#include "GeometryTools.h"

extern "C"{
#include "network_cfg.h"
#include "botNet_core.h"
}

Servo::Servo() {
    // TODO Auto-generated constructor stub

}

Servo::~Servo() {
    // TODO Auto-generated destructor stub
}


int Servo::sendPosServo(eServos s, float angle /* deg */) {
    sMsg msg = { { 0 } };

    msg.header.destAddr = ADDRI_MAIN_IO;
    msg.header.type = E_SERVOS;
    msg.payload.servos.nb_servos = 1;
    msg.header.size = 2 + 5*msg.payload.servos.nb_servos;
    msg.payload.servos.servos[0].id = s;
    msg.payload.servos.servos[0].angle = angle;

    bn_sendRetry(&msg, MAX_RETRIES_SERVO);

    return 1;
}
