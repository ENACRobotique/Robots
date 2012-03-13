#include <Arduino.h>

#include <MsTimer2.h>
#include "main.hpp"
extern "C" {
#include "pos_estimate.h"
}

#include "suivi_ligne.hpp"

void periodic_line();

void init_line() {
    MsTimer2::set(10, periodic_line);    // 10ms periodic task
    MsTimer2::start();
}

void deinit_line() {
    MsTimer2::stop();
}

#ifdef DEBUG
unsigned int _pos_c;
#endif

void periodic_line() {
    bool ligLft, ligRgt;
    int speed=50, angle=0;
    static ePosition pos_c=pLOST;

// read sensors data (HIGH => on the line, LOW => on the blue)
    ligLft = digitalRead(ligPinLft);
    ligRgt = digitalRead(ligPinRgt);

// position estimation
    pos_c=(ePosition)pos_next[POS(pos_c)][ligLft][ligRgt];

    switch(POS(pos_c)) {
    case pLOST:
                        angle = 0;      speed = 75;
        break;
    case pLEFT:
                        angle = -40;    speed = 100;
        break;
    case pLEFT_CENTER:
                        angle = -15;    speed = 150;
        break;
    case pCENTER:
                        angle = 0;      speed = 150;
        break;
    case pRIGHT_CENTER:
                        angle = +15;    speed = 150;
        break;
    case pRIGHT:
                        angle = +40;    speed = 100;
        break;
    default:
        pos_c = (ePosition)(pLOST|pWTF);
        break;
    }

    servoDir.write(CLAMP(0, angle+45, 45*2));    // sets the servo position (45 is ~neutral)
    
// set speed
    digitalWrite(motorPinDir, HIGH);    // direction: forward
    analogWrite(motorPinPwm, speed);    // go

#ifdef DEBUG
    if(!update) {
        _pos_c = pos_c;

        update=1;   // tell main task the data has been updated
    }
#endif
}

