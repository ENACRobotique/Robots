#include "state_hardinit.h"

#include "state_types.h"
#include "../params.h"
#include "Arduino.h"
#include "Wire.h"
#include "lib_IHM.h"

#include "states/state_Menu_principal.h"

sState* reBlink(){
    return &sMenu_principal;
}

void update_encoder(){
	myEnc.update();
}

void initHard(sState *prev){

	startLcd();
	attachInterrupt(1,update_encoder,CHANGE);
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
