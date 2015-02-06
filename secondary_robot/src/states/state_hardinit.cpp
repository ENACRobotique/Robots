#include "state_hardinit.h"

#include "state_types.h"
#include "../params.h"
#include "Arduino.h"
#include "Wire.h"

#include "state_dead.h"
#include "state_wall.h"
#include "state_ALACON.h"
#include "state_traj.h"
#include "state_Manualdrive.h"
#include "state_tirette.h"


#include "../libs/lib_move.h"
#include "../libs/lib_motor.h"
#include "../libs/lib_radar.h"
#include "lib_wall.h"

sState* reTirette(){
	//return &sTrajRedInit;
	//return &sManualdrive;
	return &sTirette;
}
void initHard(sState *prev){

#ifdef DEBUG
    Serial.println("debut init matérielles");
#endif
    //movements
    int pin_motors_dir[NB_MOTORS];
    int pin_motors_pwm[NB_MOTORS];
    pin_motors_dir[0]=PIN_MOTOR1_DIR;
    pin_motors_pwm[0]=PIN_MOTOR1_PWM;
    pin_motors_dir[1]=PIN_MOTOR2_DIR;
    pin_motors_pwm[1]=PIN_MOTOR2_PWM;
    motorInitHard(pin_motors_dir,pin_motors_pwm);
    int pin_odo_int[NB_MOTORS]={PIN_ODO1_INT,PIN_ODO2_INT};
    int pin_odo_sen[NB_MOTORS]={PIN_ODO1_SEN,PIN_ODO2_SEN};
    odoInitHard(pin_odo_int,pin_odo_sen);

    //radar
    Wire.begin();

    //line following/detector
    //Wire.begin(); already done

    //tirette
    pinMode( PIN_TIRETTE,INPUT_PULLUP);

    //"color" (start side) button
    pinMode(PIN_COLOR,INPUT_PULLUP);


#ifdef DEBUG
    Serial.println("fin init matérielles");
#endif

}
sState sInitHard={
    0,
    &initHard,
    NULL,
    &reTirette
};
