#include "state_hardinit.h"

#include "state_types.h"
#include "../params.h"
#include "Arduino.h"
#include "Wire.h"

#include "state_dead.h"
#include "state_ALACON.h"
#include "state_traj.h"
#include "state_Manualdrive.h"
#include "state_tirette.h"
#include "state_traj.h"

#include "../libs/lib_attitude.h"
#include "../libs/MPU_6050.h"
#include "../libs/lib_move.h"
#include "../libs/lib_motor.h"
#include "../libs/lib_radar.h"
#include "lib_wall.h"

sState* reTirette(){
//	return &sTirette;
//	return &sAlacon;
    return &sTrajGreenInit;
}

void initHard(sState *prev){

#ifdef DEBUG
    Serial.println("debut init matérielles");
#endif
    //movements
    int pin_motors_dir[NB_MOTORS];
    int pin_motors_pwm[NB_MOTORS];
    pin_motors_dir[0]=PIN_MOTOR_DIR;
    pin_motors_pwm[0]=PIN_MOTOR_PWM;
    motorInitHard(pin_motors_dir,pin_motors_pwm);
    int pin_odo_int[NB_MOTORS]={PIN_ODO_INT};
    int pin_odo_sen[NB_MOTORS]={PIN_ODO_SEN};
    odoInitHard(pin_odo_int,pin_odo_sen);

    //radar
    //Wire.begin();
    //initInertial();
    //servoInitHard(PIN_SERVO_ATTITUDE);
    //line following/detector
    //Wire.begin(); already done
    //fan


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
