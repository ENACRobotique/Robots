#include <stdlib.h>
#include <Arduino.h>

#include "main.hpp"

#include "parallel.hpp"

typedef struct {
    int speed;
    int angle;
    unsigned int time;
} sMove;

sMove parallel_tab[] = {
    { -70, 0, 1100 },
    { +70, 110, 3800 },
    { 0, 0, 100 }
};

int step;
long start;

void init_parallel() {
    step=0;
    start=millis();
}

void deinit_parallel() {
    
}

int step_parallel() {
    if(start+parallel_tab[step].time<millis()) {
        step++;

        if(step>=sizeof(parallel_tab)/sizeof(*parallel_tab))
            return 1;

        start=millis();
    }

    servoDir.write(CLAMP(0, parallel_tab[step].angle+45, 45*2));    // sets the servo position (45 is ~neutral)

    if(parallel_tab[step].speed<0) {
        digitalWrite(motorPinDir, LOW);    // direction: backward
        analogWrite(motorPinPwm, -parallel_tab[step].speed);    // go
    }
    else if(parallel_tab[step].speed>=0) {
        digitalWrite(motorPinDir, HIGH);    // direction: forward
        analogWrite(motorPinPwm, parallel_tab[step].speed);    // go
    }

    return 0;
}

