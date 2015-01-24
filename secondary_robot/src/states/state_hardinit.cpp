#include "state_hardinit.h"

#include "state_types.h"
#include "../params.h"
#include "Arduino.h"
#include "Wire.h"

#include "state_dead.h"
#include "state_wall.h"
#include "state_ALACON.h"
#include "state_traj.h"


#include "../libs/lib_move.h"
#include "../libs/lib_motor.h"
#include "../libs/lib_radar.h"
#include "lib_wall.h"

sState* reTirette(){
    return &sTrajRedInit;
}
void initHard(sState *prev){

#ifdef DEBUG
    Serial.println("debut init matérielles");
#endif
    //movements
    motorInitHard(PIN_MOTOR1_DIR,PIN_MOTOR1_PWM);
    odoInitHard(PIN_ODO1_INT,PIN_ODO1_SEN);

    //radar
    Wire.begin();

    //line following/detector
    //Wire.begin(); already done

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
