#include <Arduino.h>

#include <MsTimer2.h>
#include "main.hpp"
extern "C" {
#include "sharp_2d120x.h"
}

#include "suivi_mur.hpp"

void periodic_wall();

void init_wall() {
    MsTimer2::set(30, periodic_wall);    // 30ms periodic task
    MsTimer2::start();
}

void deinit_wall() {
    MsTimer2::stop();

    analogWrite(motorPinPwm, 0);
}

#ifdef DEBUG
unsigned int _dist;
#endif

void periodic_wall() {
    unsigned int dist;
    static unsigned int last_dist=0;
    static int I=0;
    int tmp, D;

    // low-pass filter (from http://www.edn.com/contents/images/6335310.pdf)
    #define FILTER_SHIFT 2
    static unsigned int filter_reg = 0;

// get distance from Sharp sensor
    // get raw data and apply a low-pass filter
    filter_reg = filter_reg - (filter_reg>>FILTER_SHIFT) + analogRead(optPinDis);
    dist = filter_reg>>FILTER_SHIFT;

    // convert raw data to centimeters<<4 (cf sharp_2d120x.c/h)
    dist=raw2dist(dist);

// compute epsilon
    tmp = dist-(20<<4); // the set point is 20 centimeters

// compute integral term by successive sums
    I = CLAMP(-(64<<4), I+tmp, 64<<4);  // clamp the integral term to limit overrun (TODO: find good limits)

// compute derivative term
    D = (dist - last_dist)<<5;  // <<5 is ~1/dt (dt is ~30ms)
    last_dist = dist;

// Ziegler-Nichols: Ku~=18, Tu~=1s
//    tmp = -11*tmp + -((22*I)>>5) + -2*D;  // PID: Kp*epsilon + Ki*I + Kd*D (>>5 is ~dt)
//    tmp = -8*tmp + -((10*I)>>5);          // PI:  Kp*epsilon + Ki*I (>>5 is ~dt)
//    tmp = -9*tmp;                         // P:   Kp*epsilon

//  - pour mur à droite, + pour mur à gauche
    tmp = -8*tmp;                           // P:   Kp*epsilon

    servoDir.write(CLAMP(0, (tmp>>4)+45, 45*2));    // sets the servo position (45 is ~neutral)

// set speed
    digitalWrite(motorPinDir, HIGH);    // direction: forward
    analogWrite(motorPinPwm, 160);    // go

#ifdef DEBUG
    update=1;   // tell main task the data has been updated
#endif
}

