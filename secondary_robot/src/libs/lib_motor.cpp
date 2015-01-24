/*
this library contains the different functions useful for the motor and its control
*/

#include "lib_motor.h"

//defines
#define MOT_ASSER_PERIOD 20 // milliseconds

#ifndef CLAMP
#define CLAMP(m, n, M) min(max((m), (n)), (M))
#endif

//globals
int _motCon;
int _motPinDir,_motPinPWM;

//initializes the PINs for motor speed and direction
//REQUIRES : odoinitHard()
void motorInitHard(int pinDir,int pinPWM){
    _motPinDir=pinDir;
    _motPinPWM=pinPWM;
    pinMode(pinDir, OUTPUT);  // the direction pin of the motor is an output
    pinMode(pinPWM, OUTPUT);  // the PWM pin of the motor is an output
    analogWrite(_motPinPWM,0);
}

#define KP  18// >>2 , with ziegler nichols (Ku = 9>>2, Tu=80ms)
#define KI  5// >>2

void motAsser(){
    unsigned long int time=millis();
    int eps;
    static int intEps=0;
    static unsigned long time_prev_asser=millis();
    static int _motCmd=0;

    // asservissement vitesse
    if((time-time_prev_asser)>=MOT_ASSER_PERIOD) {
        if ( (time-time_prev_asser) < MOT_ASSER_PERIOD+MOT_ASSER_PERIOD/2 ){
            time_prev_asser = time_prev_asser + MOT_ASSER_PERIOD;
            //compute error (epsilon)
            int read = odoRead();
            eps = _motCon - read;//odoRead is negative if the robot is going forward "red side"

            //compute error integral
            intEps= CLAMP( -(64<<4) ,intEps+eps, (64<<4));
            //compute command
            if(_motCon==0){
            _motCmd=0;
            }
            else{
            	_motCmd=  ((KP*eps)>>2) + ((KI*intEps)>>2);
            }

#ifdef DEBUG_MOTOR
Serial.print(_motCon);
Serial.print("\t");
Serial.print(read);
Serial.print("\t");
Serial.print(_motCmd);
Serial.print("\t");
Serial.print(CLAMP(0,abs(_motCmd),254));
Serial.print("\t");
Serial.print(eps);
Serial.print("\t");
Serial.print(intEps);
Serial.print("\t");
Serial.println(millis());
#endif
            if(_motCmd>=0) digitalWrite(_motPinDir,LOW);
            else digitalWrite(_motPinDir,HIGH);

            analogWrite(_motPinPWM,CLAMP(0,abs(_motCmd),254));
        }
        else {//to avoid problems due to long loop
              time_prev_asser=millis();
              intEps=0;//<=>resets the integral term
              analogWrite(_motPinPWM, CLAMP(0,_motCmd,254));
              odoRead();
            }
    }
}
