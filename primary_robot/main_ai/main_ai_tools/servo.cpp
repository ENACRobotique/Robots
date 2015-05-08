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



//int Servo::sendPosServo(eServos /*s*/, int16_t /*us*/, int16_t /*a*/) { // us or a = -1 if no use
//    sMsg msg;
//    Point2D<float> p1, p2;
//    int i = 0;
//
//    if (((us == -1) && (a == -1)) || ((us != -1) && (a != -1))) {
//        return -1;
//    }
//
//    if (a != -1) {
//        i = 0;
//        while (s != _servo.id) {
//            i++;
//          //  if (i > sizeof(_servo) / sizeof(*_servo)) //FIXME
//                break;
//        }
//        p1.x = _servo.a1;
//        p1.y = _servo.u1;
//        p2.x = _servo.a2;
//        p2.y = _servo.u2;
//        Line2D<float> l(p1, p2);
//
//        if (l.b == 0)
//            return -1;
//
//        us = -(l.a * a + l.c) / l.b;
//    }
//
//    msg.header.destAddr = ADDRI_MAIN_IO;
//    msg.header.type = E_SERVOS;
//    msg.header.size = 2 + 1 * sizeof(msg.payload.servos.servos[0]);
//    msg.payload.servos.nb_servos = 1;
//    msg.payload.servos.servos[0].id = s;
//    msg.payload.servos.servos[0].us = us;
//
//    bn_sendRetry(&msg, MAX_RETRIES_SERVO);
/*
int Servo::sendPosServo(eServos s, float angle / deg /) {
    sMsg msg;

    msg.header.destAddr = ADDRI_MAIN_IO;
    msg.header.type = E_SERVOS;
    msg.payload.servos.nb_servos = 1;
    msg.header.size = 2 + 5*msg.payload.servos.nb_servos;
    msg.payload.servos.servos[0].id = s;
    msg.payload.servos.servos[0].angle = angle;

    bn_sendRetry(&msg, MAX_RETRIES_SERVO);
    return 1;
}
*/
