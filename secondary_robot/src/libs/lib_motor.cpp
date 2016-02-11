/*
this library contains the different functions useful for the motor and its control
*/

#include "lib_motor.h"

//defines
#define MOT_ASSER_PERIOD 20 // milliseconds
#define AMAX 50
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
int Kp[NB_MOTORS] ={11};
int Ki[NB_MOTORS] = {2}; // >>2
#elif NB_MOTORS == 2
int Kp[NB_MOTORS] ={18,18}; // >>2
int Ki[NB_MOTORS] = {5,5}; // >>2
#endif
void motAsser(){
    unsigned long int time=millis();
    static int intEps[NB_MOTORS]={0};
    static unsigned long time_prev_asser=0;
    static int _motCmd[NB_MOTORS]={0};
    static int motCurrentCon[NB_MOTORS]={0};


	int eps;
	// asservissement vitesse
	if((time-time_prev_asser)>=MOT_ASSER_PERIOD) {
		if ( (time-time_prev_asser) < MOT_ASSER_PERIOD+(MOT_ASSER_PERIOD>>1) ){
			motGetCon(motCurrentCon);
			time_prev_asser = time_prev_asser + MOT_ASSER_PERIOD;
			for(int i=0;i<NB_MOTORS;i++)
			{
				//compute error (epsilon)
				int read=odoRead(i);
				eps = motCurrentCon[i] - read;//odoRead is negative if the robot is going forward "red side"

				//compute error integral
				intEps[i]= CLAMP( -(64<<5) ,intEps[i]+eps, (64<<5));
				//compute command
				if(motCurrentCon[i]==0){
				_motCmd[i]=0;
				}
				else{
					_motCmd[i]=  ((Kp[i]*eps)>>4) + ((Ki[i]*intEps[i])>>5);
				}

	#ifdef DEBUG_MOTOR
	Serial.print(_motCon[i]);
	Serial.print("\t");
	Serial.print(motCurrentCon[i]);
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
		}
		else {//to avoid problems due to long loop
			for(int i = 0; i< NB_MOTORS;i++){
				  time_prev_asser=millis();
				  intEps[i]=0;//<=>resets the integral term
				  analogWrite(_motPinPWM[i], CLAMP(0,abs(_motCmd[i]),254));
				  odoRead(i);
			}
		}
	}
}

void motGetCon(int *rc)//Create a new con with the last one
{
	int dc[NB_MOTORS]={0};
	for(int i=0;i<NB_MOTORS;i++){
		dc[i]=rc[i]-_motCon[i];
		int dcAbs=min(AMAX,abs(dc[i]));
		if(dc[i]<0)
		{
			rc[i]+=dcAbs;
		}
		else
		{
			rc[i]-=dcAbs;
		}
	}
}
