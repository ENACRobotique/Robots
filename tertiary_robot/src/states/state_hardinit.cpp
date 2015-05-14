#include "state_hardinit.h"

#include "state_types.h"
#include "params.h"
#include "Arduino.h"
#include "Wire.h"

#include "state_tirette.h"


#include "lib_move.h"
#include "lib_motor.h"
#include "lib_radar.h"
#include "lib_wall.h"

sState* reTirette(){
    return &sTirette;
}
void initHard(sState *prev){

#ifdef DEBUG
    Serial.println("debut init matérielles");
#endif
    //movements
    //movements
    int pin_motors_dir[NB_MOTORS];
    int pin_motors_pwm[NB_MOTORS];
    pin_motors_dir[0]=PIN_MOTOR1_DIR;
    pin_motors_pwm[0]=PIN_MOTOR1_PWM;
    motorInitHard(pin_motors_dir,pin_motors_pwm);
    int pin_odo_int[NB_MOTORS]={PIN_ODO1_INT};
    int pin_odo_sen[NB_MOTORS]={PIN_ODO1_SEN};
    odoInitHard(pin_odo_int,pin_odo_sen);

    //radar
    Wire.begin();

    //line following/detector
    //Wire.begin(); already done

    //launcher
    pinMode(PIN_LAUNCHER_1,OUTPUT);
    pinMode(PIN_LAUNCHER_2,OUTPUT);
    pinMode(PIN_LAUNCHER_NET,OUTPUT);
    launcherServoUp.attach(PIN_LAUNCHER_1);
    launcherServoDown.attach(PIN_LAUNCHER_2);
    launcherServoNet.attach(PIN_LAUNCHER_NET);



    //wall
    wallInitHard(PIN_SHARP_FRONT_RIGHT ,PIN_SHARP_BACK_RIGHT,PIN_SHARP_FRONT_LEFT  ,PIN_SHARP_BACK_LEFT);

    //tirette
    pinMode( PIN_TIRETTE,INPUT_PULLUP);

    //"color" (start side) button
    pinMode(PIN_COLOR,INPUT_PULLUP);

    //led pin
    pinMode( PIN_LED , OUTPUT);
    digitalWrite(PIN_LED,LOW);

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
