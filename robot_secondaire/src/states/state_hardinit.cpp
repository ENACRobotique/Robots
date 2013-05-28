#include "state_hardinit.h"

#include "state_types.h"
#include "../params.h"
#include "Arduino.h"
#include "Wire/Wire.h"

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
    radarInitHard(PIN_RAD_SERVO);
    Wire.begin();

    //line following/detector
    //Wire.begin(); already done

    //funny
    pinMode( PIN_FUNNY , OUTPUT);
    digitalWrite(PIN_FUNNY,LOW);

    //arms
    pinMode(PIN_ARM_LEFT,OUTPUT);
    pinMode(PIN_ARM_RIGHT,OUTPUT);
    armServoLeft.attach(PIN_ARM_LEFT);
    armServoLeft.write(ARM_LEFT_UP);


    //wall
    wallInitHard(PIN_SHARP_LEFT,PIN_SHARP_RIGHT);

    //tirette
    pinMode( PIN_TIRETTE,INPUT);

    //"color" (start side) button
    pinMode(PIN_COLOR,INPUT);

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
