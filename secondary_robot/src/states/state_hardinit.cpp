#include "state_hardinit.h"

#include "state_types.h"
#include "../params.h"
#include "Arduino.h"
#include "Wire.h"

#include "state_dead.h"
#include "state_stairs.h"
#include "state_traj.h"
#include "state_tirette.h"

#include "../libs/lib_attitude.h"
#include "../libs/MPU_6050.h"
#include "../libs/lib_move.h"
#include "../libs/lib_motor.h"
#include "../libs/lib_radar.h"
#include "../libs/lib_fan.h"
#include "lib_wall.h"

sState* reTirette(){
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
    motorInitHard(pin_motors_dir,pin_motors_pwm);
    int pin_odo_int[NB_MOTORS]={PIN_ODO1_INT};
    int pin_odo_sen[NB_MOTORS]={PIN_ODO1_SEN};
    odoInitHard(pin_odo_int,pin_odo_sen);
    moveInitHard(PIN_SERVO_DIR,ANGLE_ZERO,0);
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
