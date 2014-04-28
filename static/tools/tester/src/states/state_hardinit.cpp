#include "state_hardinit.h"

#include "state_types.h"
#include "../params.h"
#include "Arduino.h"
#include "Wire.h"

#include "states/state_blink.h"
#include "states/state_Menu_principal.h"
#include "states/state_Menu_servo.h"
#include "states/state_servo_selecter1.h"
#include "states/state_servo_selecter2.h"

sState* reBlink(){
    return &sMenu_principal;
}
void initHard(sState *prev){

#ifdef DEBUG
    Serial.println("debut init matérielles");
#endif

#ifdef DEBUG
    Serial.println("fin init matérielles");
#endif

}
sState sInitHard={
    0,
    &initHard,
    NULL,
    &reBlink
};
