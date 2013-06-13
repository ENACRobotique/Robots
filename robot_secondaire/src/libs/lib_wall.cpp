/*
 * lib_wall.cpp
 *
 *  Created on: 15 mai 2013
 *      Author: quentin
 */



#include "Servo/Servo.h"
#include "lib_move.h"
#include "lib_wall.h"
#include "sharp_2d120x.h"


#ifndef CLAMP
#define CLAMP(m, n, M) min(max((m), (n)), (M))
#endif

int _wall_pin_left,_wall_pin_right;
int wall_side, wall_dist, wall_speed=0;

void wallSetVal(int side, int dist, int speed){
    wall_side=side;
    wall_dist=dist;
    wall_speed=speed;
}


void wallInitHard(int left_sensor, int right_sensor){
    pinMode(left_sensor,INPUT);
    pinMode(right_sensor,INPUT);
    _wall_pin_left=left_sensor;
    _wall_pin_right=right_sensor;
}


#define KPL 16 // >>6
#define KDL 4 // >>5
#define KIL 0 // >>9
#define KPR 16 // >>6
#define KDR 4 // >>5
#define KIR 0 // >>9
#define WALL_PERIOD 31

void periodicWall(){
    unsigned int dist;
    static unsigned long int prevMillis=0;
    static int I=0, D=0, last_dist;
    int tmp;
    unsigned long int dtime=millis()-prevMillis;
    if ( dtime >=WALL_PERIOD){
        if (dtime>=WALL_PERIOD + WALL_PERIOD/2  ) {
            prevMillis=millis();
            return;
        }
        prevMillis=millis();

        // low-pass filter (from http://www.edn.com/contents/images/6335310.pdf)
        #define FILTER_SHIFT 2
        static unsigned int filter_reg = 0;

        // get distance from Sharp sensor
        // get raw data and apply a low-pass filter*
        if (wall_side==LEFT) filter_reg = filter_reg - (filter_reg>>FILTER_SHIFT) + analogRead(_wall_pin_left);
        else filter_reg = filter_reg - (filter_reg>>FILTER_SHIFT) + analogRead(_wall_pin_right);
        dist = filter_reg>>FILTER_SHIFT;
        #undef FILTER_SHIFT
        // convert raw data to centimeters<<4 (cf sharp_2d120x.c/h)
        dist=raw2dist120x(dist);

        // compute epsilon
        tmp = dist-(wall_dist<<3);

        // compute integral term by successive sums
        I = CLAMP(-(64<<4), I+tmp, 64<<4);  // clamp the integral term to limit overrun (TODO: find good limits)

        // compute derivative term
            D = (dist - last_dist)<<5;  // <<5 is ~1/dt (dt is ~30ms)
            last_dist = dist;

        // Ziegler-Nichols: Ku~=18, Tu~=1s
        //    tmp = -11*tmp + -((22*I)>>5) + -2*D;  // PID: Kp*epsilon + Ki*I + Kd*D (>>5 is ~dt)
        //    tmp = -8*tmp + -((10*I)>>5);          // PI:  Kp*epsilon + Ki*I (>>5 is ~dt)
        //    tmp = -9*tmp;                         // P:   Kp*epsilon


        // signe : fonction du côté
        if (wall_side==LEFT) {
            tmp = -((KPL*tmp)>>6)-((KIL*I)>>9)-((KDL*D)>>5);                           // P:   Kp*epsilon
        }
        else tmp = +((KPR*tmp)>>6)+((KIR*I)>>9)+((KDR*D)>>5);
        move(wall_speed,CLAMP(-30,tmp,30));

    }

}

