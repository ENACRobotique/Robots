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
int _motCon[NB_MOTORS];
int _motPinDir[NB_MOTORS],_motPinPWM[NB_MOTORS];

//initializes the PINs for motor speed and direction
//REQUIRES : odoinitHard()
void motorInitHard(int pinDir[],int pinPWM[]){
	for(int i=0;i<NB_MOTORS;i++)
	{
		_motPinDir[i]=pinDir[i];
		pinMode(pinDir[i], OUTPUT);  // the direction pin of the motor is an output
	}

	for(int i=0;i<NB_MOTORS;i++)
	{
		_motPinPWM[i]=pinPWM[i];
		pinMode(pinPWM[i], OUTPUT);  // the PWM pin of the motor is an output
		analogWrite(pinPWM[i],0);
	}

}
#if NB_MOTORS == 1
int Kp[NB_MOTORS] ={12};
int Ki[NB_MOTORS] = {30}; // >>2
#elif NB_MOTORS == 2
int Kp[NB_MOTORS] ={18,18}; // >>2
int Ki[NB_MOTORS] = {5,5}; // >>2
#endif
void motAsser(){
    unsigned long int time=millis();
    static int intEps[NB_MOTORS]={0};
    static unsigned long time_prev_asser[NB_MOTORS]={0};
    static int _motCmd[NB_MOTORS]={0};

    for(int i=0;i<NB_MOTORS;i++)
    {
		int eps;
		// asservissement vitesse
		if((time-time_prev_asser[i])>=MOT_ASSER_PERIOD) {
			if ( (time-time_prev_asser[i]) < MOT_ASSER_PERIOD+(MOT_ASSER_PERIOD>>1) ){
				time_prev_asser[i] = time_prev_asser[i] + MOT_ASSER_PERIOD;
				//compute error (epsilon)
				int read=odoRead(i);
				eps = _motCon[i] - read;//odoRead is negative if the robot is going forward "red side"

				//compute error integral
				intEps[i]= CLAMP( -(64<<4) ,intEps[i]+eps, (64<<4));
				//compute command
				if(_motCon[i]==0){
				_motCmd[i]=0;
				}
				else{
					_motCmd[i]=  ((Kp[i]*eps)) + ((Ki[i]*intEps[i])>>2);
				}

	#ifdef DEBUG_MOTOR
	Serial.print(_motCon[i]);
	Serial.print("\t");
	Serial.print(read);
	Serial.print("\t");
	Serial.print(_motCmd[i]);
	Serial.print("\t");
	Serial.print(CLAMP(0,abs(_motCmd[i]),254));
	Serial.print("\t");
	Serial.print(eps);
	Serial.print("\t");
	Serial.print(intEps[i]);
	Serial.print("\t");
	Serial.println(millis());
	#endif
				if(_motCmd[i]>=0) digitalWrite(_motPinDir[i],LOW);
				else digitalWrite(_motPinDir[i],HIGH);

				analogWrite(_motPinPWM[i],CLAMP(0,abs(_motCmd[i]),254));
			}
			else {//to avoid problems due to long loop
				  time_prev_asser[i]=millis();
				  intEps[i]=0;//<=>resets the integral term
				  analogWrite(_motPinPWM[i], CLAMP(0,abs(_motCmd[i]),254));
				  odoRead(i);
				}
		}
	}
}
