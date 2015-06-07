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
#include "tools.h"

int Servo::sendPosServo(const servoName name, const float angle) {
    sMsg msg;

    memset(&msg, 0, sizeof(msg));

    logs << INFO << "Send servo position simple:" << name;
    if(angle < 0)
        return -1;

    logs << INFO << "tabServo simple:";
    for(unsigned int i = 0 ; i < 13 ; i++){
        logs << "\n name:" << unsigned(tabServo[(servoName) i].club_id) << " hw:" << unsigned(tabServo[(servoName) i].hw_id) ;
    }

    msg.header.destAddr = ADDRI_MAIN_IO;
    msg.header.type = E_SERVOS;
    //msg.header.size = 2 + 6*msg.payload.servos.nb_servos;
    msg.header.size = 2 + 1 * sizeof(msg.payload.servos.servos[0]);
    msg.payload.servos.nb_servos = 1;
    msg.payload.servos.servos[0].club_id = tabServo[name].club_id;
    msg.payload.servos.servos[0].hw_id = tabServo[name].hw_id;
    msg.payload.servos.servos[0].angle = angle;

    bn_sendRetry(&msg, MAX_RETRIES_SERVO);

    logs << INFO << "Send servo position id_club:" << unsigned(tabServo[name].club_id) << "ang:" << angle;
    return 1;
}

int Servo::sendMultiPosServo(const std::vector<servoName> name, const std::vector<float> angle){
    sMsg msg;

    memset(&msg, 0, sizeof(msg));

    logs << ERR << "name id" << unsigned(tabServo[servoName::PINCE_UP_DOWN1].club_id);
    logs << ERR << "name id" << unsigned(tabServo[name[0]].club_id);
    printf("pri%u", tabServo[name[0]].club_id);
    logs << INFO << "tabServomulti:";
    for(unsigned int i = 0 ; i < 13 ; i++){
        logs << "\n name:" << unsigned(tabServo[(servoName) i].club_id) << " hw:" << unsigned(tabServo[(servoName) i].hw_id) ;
    }

    if(name.size() != angle.size()){
        logs << ERR << "Different size servo";
        return -1;
    }

    if(name.size() > 6){
        logs << ERR << "maximum servo number";
        return -2;
    }


    msg.header.destAddr = ADDRI_MAIN_IO;
    msg.header.type = E_SERVOS;
    msg.header.size = 2 + name.size()*sizeof(msg.payload.servos.servos[0]);

    msg.payload.servos.nb_servos = name.size();
    for(unsigned int i = 0 ; i < name.size() ; i++){
        msg.payload.servos.servos[i].club_id = tabServo[name[i]].club_id;
        msg.payload.servos.servos[i].hw_id = tabServo[name[i]].hw_id;
        msg.payload.servos.servos[i].angle = angle[i];
    }

    bn_sendRetry(&msg, MAX_RETRIES_SERVO);
    logs << INFO << "Send servo postion";
    return 1;
}

