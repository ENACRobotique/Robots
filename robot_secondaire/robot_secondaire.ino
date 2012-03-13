#include <Servo.h>  // shipped with arduino-core
#include <MsTimer2.h>   // get this library at http://arduino.cc/playground/Main/MsTimer2

#include "suivi_ligne.hpp"
#include "suivi_mur.hpp"
#include "states.hpp"
#include "parallel.hpp"

#include "main.hpp"

// input
const int ligPinLft = 2;    // gpio
const int ligPinRgt = 3;    // gpio

const int optPinDis = A1;    // analog

// output
Servo servoDir;    // create servo object to control a servo
const int servoPinDir = 5;    // servo(pwm)     Timer1A
const int motorPinDir = 7;    // gpio
const int motorPinPwm = 6;    // pwm            Timer1B

// Rq. Timer0 is used for millis/micros

void setup() {
#ifdef DEBUG
    // debug
    Serial.begin(19200);
#endif

    // input
    pinMode(ligPinLft, INPUT);
    pinMode(ligPinRgt, INPUT);
    pinMode(4, INPUT);  // message

    // output
    pinMode(13, OUTPUT);    // debug blinking led
    pinMode(motorPinDir, OUTPUT);
    servoDir.attach(servoPinDir);    // attaches the servo on pin servoPinDir to the servo object
}

// input
//    analogRead(analogPin)==0 -> 1023
//    digitalRead(gpioPin)==LOW | HIGH

// output
//    digitalWrite(gpioPin, LOW | HIGH);
//    servoObj.write(0 -> 180);
//    analogWrite(pwmPin, 0 -> 255);

#ifdef DEBUG
volatile unsigned int update=0;
#endif

eStates curr_state=sINIT, prev_state=sINIT;
long start_state=0;
#define TIME_STATE (millis()-start_state)

void loop() {
    static long tempo_led=0;
    static int etat_led=0;

    if(millis()-tempo_led>100) {
        tempo_led=millis();
        etat_led^=1;
        digitalWrite(13, etat_led);
    }

#ifdef DEBUG
    Serial.print(millis());
    Serial.print(",");
    Serial.print(TIME_STATE);
    Serial.print(",");
    Serial.print(prev_state);
    Serial.print(",");
    Serial.print(curr_state);
#endif

    switch(curr_state) {
    case sINIT:
        curr_state=sLINE;
        break;
    case sLINE:
#ifdef DEBUG
        if(update) {
            Serial.print(",");
            Serial.print(_pos_c);
        }
#endif

        if(TIME_STATE>8650) // TODO: check current consumption of the motor
            curr_state=sPARALLEL;
        break;
    case sPARALLEL: // TODO: create a generic state to handle programmed trajectories
        if(step_parallel())
            curr_state=sWALL;
        break;
    case sWALL:
#ifdef DEBUG
        if(update) {
            Serial.print(",");
            Serial.print(_dist);
        }
#endif
        if(TIME_STATE>4500) // TODO: detect 2nd bottle message
            curr_state=sSTOP;
        break;
    case sSTOP:
    default:
        break;
    }

    // TODO make a struct defining a state (val, init, step, deinit, ...)
    if(prev_state!=sLINE && curr_state==sLINE) {    // init line follow
        init_line();
    }

    if(curr_state!=sLINE && prev_state==sLINE) {
        deinit_line();
    }

    if(prev_state!=sPARALLEL && curr_state==sPARALLEL) {
        init_parallel();
    }

    if(curr_state!=sPARALLEL && prev_state==sPARALLEL) {
        deinit_parallel();
    }

    if(prev_state!=sWALL && curr_state==sWALL) {    // init wall follow
        init_wall();
    }

    if(curr_state!=sWALL && prev_state==sWALL) {
        deinit_wall();
    }

    if(curr_state != prev_state)
        start_state=millis();

    prev_state = curr_state;

#ifdef DEBUG
    if(update)
        update=0;
    Serial.print("\r\n");
#endif
}

