/*
 * lib_wall.cpp
 *
 *  Created on: 15 mai 2013
 *      Author: quentin
 */



#include "Servo.h"
#include "lib_move.h"
#include "lib_wall.h"
#include "sharp_2d120x.h"


#ifndef CLAMP
#define CLAMP(m, n, M) min(max((m), (n)), (M))
#endif

int _wall_pin_left_front ,_wall_pin_left_back,_wall_pin_right_front,_wall_pin_right_back;
int wall_side, wall_dist, wall_speed=0;

void wallSetVal(int side, int dist, int speed)
	{
    wall_side=side;
    wall_dist=dist;
    wall_speed=speed;
	}


void wallInitHard(int right_sensor_front ,  int right_sensor_back, int left_sensor_front, int left_sensor_back ){
    pinMode(right_sensor_front  ,INPUT);
    pinMode(right_sensor_back,INPUT);

    pinMode(left_sensor_front,INPUT);
    pinMode(left_sensor_back ,INPUT);

    _wall_pin_right_front =right_sensor_front;
    _wall_pin_right_back =right_sensor_back;

    _wall_pin_left_front =left_sensor_front;
    _wall_pin_left_back =left_sensor_back;


}


#define KPL 16 // >>6
#define KDL 4 // >>5
#define KIL 0 // >>9
#define KPR 16 // >>6
#define KDR 4 // >>5
#define KIR 0 // >>9
#define WALL_PERIOD 31

void periodicWall(){
    unsigned int dist,dist2;
    static unsigned long int prevMillis=0;
    static int I=0,I2=0 , D=0, D2=0, last_dist , last_dist2;
    int tmp,tmp2;
//    int val;
    unsigned long int dtime=millis()-prevMillis;

    if ( dtime >=WALL_PERIOD){
        if (dtime>=WALL_PERIOD + WALL_PERIOD/2  ) {
            prevMillis=millis();
            return;
        }
        prevMillis=millis();

        // low-pass filter (from http://www.edn.com/contents/images/6335310.pdf)
        #define FILTER_SHIFT 2
		#define FILTER_SHIFT2 2
        static unsigned int filter_reg = 0 , filter_reg2 = 0;

        // get distance from Sharp sensor
        // get raw data and apply a low-pass filter*
        if (wall_side==LEFT) {filter_reg = filter_reg - (filter_reg>>FILTER_SHIFT) + analogRead(_wall_pin_left_front);
        					  filter_reg2 = filter_reg2 - (filter_reg2>>FILTER_SHIFT2) + analogRead(_wall_pin_left_back);
        					 }
        else				 {filter_reg = filter_reg - (filter_reg>>FILTER_SHIFT) + analogRead(_wall_pin_right_front);
        					  filter_reg2 = filter_reg2 - (filter_reg2>>FILTER_SHIFT2) + analogRead(_wall_pin_right_back);
        					 }

        dist = filter_reg>>FILTER_SHIFT;
        dist2 = filter_reg2>>FILTER_SHIFT2;
        #undef FILTER_SHIFT

        // convert raw data to centimeters<<4 (cf sharp_2d120x.c/h)
        dist=raw2dist120x(dist);
        dist2=raw2dist120x(dist2);
        // compute epsilon

        tmp = dist-(wall_dist<<3);
        tmp2 = dist2-(wall_dist<<3);
        // compute integral term by successive sums
        I = CLAMP(-(64<<4), I+tmp, 64<<4);  // clamp the integral term to limit overrun (TODO: find good limits)
        I2 = CLAMP(-(64<<4), I2+tmp2, 64<<4);
        // compute derivative term
            D = (dist - last_dist)<<5;  // <<5 is ~1/dt (dt is ~30ms)
            D2 = (dist2 - last_dist2)<<5;
            last_dist = dist;
            last_dist2 = dist2;

        // Ziegler-Nichols: Ku~=18, Tu~=1s
        //    tmp = -11*tmp + -((22*I)>>5) + -2*D;  // PID: Kp*epsilon + Ki*I + Kd*D (>>5 is ~dt)
        //    tmp = -8*tmp + -((10*I)>>5);          // PI:  Kp*epsilon + Ki*I (>>5 is ~dt)
        //    tmp = -9*tmp;                         // P:   Kp*epsilon


        // signe : fonction du côté
        if (wall_side==LEFT) {
            tmp = -((KPL*tmp)>>6)-((KIL*I)>>9)-((KDL*D)>>5);                           // P:   Kp*epsilon
            tmp2 = -((KPL*tmp2)>>6)-((KIL*I2)>>9)-((KDL*D2)>>5);
        }
        else{ tmp = +((KPR*tmp)>>6)+((KIR*I)>>9)+((KDR*D)>>5);
              tmp2 = +((KPR*tmp2)>>6)+((KIR*I2)>>9)+((KDR*D2)>>5);
        }

        //val = (fabs(temp))-(fabs(temp2))


        if(tmp>tmp2) move(wall_speed,CLAMP(-30,tmp,30));

        else 		 move(wall_speed,CLAMP(-30,tmp2,30));
    }
}

int diff(int pin_1 , int pin_2)
	{
	int dist_1, dist_2;
	dist_1=raw2dist120x(analogRead(pin_1));
	dist_2=raw2dist120x(analogRead(pin_2));
	return(abs(dist_1-dist_2));
	}

int endWall(int pin_front , int pin_back)
	{
	int front, back;
	front=raw2dist120x(analogRead(pin_front));
	back=raw2dist120x(analogRead(pin_back));
	if(front+3>back) return 1; //En cm
	return 0;
	}

