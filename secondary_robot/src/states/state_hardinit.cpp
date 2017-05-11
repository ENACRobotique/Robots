#include "state_hardinit.h"
#include "DynamixelSerial.h"
#include "state_types.h"
#include "../params.h"
#include "Arduino.h"
#include "Wire.h"

#include "state_dead.h"
#include "state_traj.h"
#include "state_tirette.h"
#include "../libs/lib_move.h"
#include "../libs/lib_motor.h"
#include "../libs/lib_radar.h"
#include "state_funny_action.h"

sState* reTirette(){
	return &sTirette;
}
void initHard(sState *prev){

#ifdef DEBUG
    Serial.println(F("debut init matérielles"));
#endif
    //movements
    int pin_motors_dir[NB_MOTORS];
    int pin_motors_pwm[NB_MOTORS];
    pin_motors_dir[0]=PIN_MOTOR1_DIR;
    pin_motors_pwm[0]=PIN_MOTOR1_PWM;
    motorInitHard(pin_motors_dir,pin_motors_pwm);
    int pin_odo_int[NB_MOTORS]={PIN_ODO1_INT};
    int pin_odo_sen[NB_MOTORS]={PIN_ODO1_SEN};
    odoInitHard(pin_odo_int,pin_odo_sen);
    moveInitHard(PIN_SERVO_DIR,ANGLE_ZERO,0);

    pinMode(PIN_POMPE_PWM, OUTPUT);
    pinMode(PIN_POMPE_DIR, OUTPUT);

    digitalWrite(PIN_POMPE_DIR,HIGH);
    analogWrite(PIN_POMPE_PWM,0);

    //radar
    Wire.begin();
#ifdef ATTITUDE
    delay(1000); // waiting for servo to reach its initial position an to stand still
#endif
    //line following/detector
    //Wire.begin(); already done

    //tirette
    pinMode( PIN_TIRETTE,INPUT_PULLUP);

    //"color" (start side) button
    pinMode(PIN_COLOR,INPUT_PULLUP);

    //Switches (wall detection)
    pinMode(PIN_SWITCH_LEFT, INPUT_PULLUP);
    pinMode(PIN_SWITCH_RIGHT, INPUT_PULLUP);


#ifdef DYN_USE
	//init DYNAMIXEL
	//initialisation => hard init
	Dynamixel.begin(1000000,DATA_DYNAMIXEL);  // Inicialize the servo at 1Mbps and Pin Control 2//action de prévention:
	Dynamixel.setTempLimit(NUM_DYNAMIXEL,80);//max 80°
	Dynamixel.setVoltageLimit(NUM_DYNAMIXEL,65,160);//6.5v=> 16v
	Dynamixel.setMaxTorque(NUM_DYNAMIXEL,512);//50%
	//action concrète
	Dynamixel.ledStatus(NUM_DYNAMIXEL,ON);
	Dynamixel.move(NUM_DYNAMIXEL,800);
#endif
	//Init servo

#ifdef DEBUG
    Serial.println(F("fin init matérielles"));
#endif

}
sState sInitHard={
    0,
    &initHard,
    NULL,
    &reTirette
};
