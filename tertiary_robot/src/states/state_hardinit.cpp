#include "state_hardinit.h"

#include "state_types.h"
#include "../params.h"
#include "Arduino.h"
#include "Wire.h"

#include "state_funny.h"
#include "state_wall.h"
#include "state_tirette.h"


#include "../libs/lib_move.h"
#include "../libs/lib_motor.h"
#include "../libs/lib_radar.h"
#include "lib_wall.h"

sState* reTirette(){
    return &sTirette;
}
void initHard(sState *prev){

#ifdef DEBUG
    Serial.println("debut init matérielles");
#endif
    //movements
    moveInitHard(PIN_DIR_SERVO, ANGLE_ZERO, DIR_SERVO_START);
    motorInitHard(PIN_MOTOR_DIR,PIN_MOTOR_PWM);
    odoInitHard(PIN_ODO_INT,PIN_ODO_SEN);

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
