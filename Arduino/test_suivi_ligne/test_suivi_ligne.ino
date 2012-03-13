#include <Servo.h>  // shipped with arduino-core
#include <MsTimer2.h>   // get this library at http://arduino.cc/playground/Main/MsTimer2

extern "C" {
#include "pos_estimate.h"
}

#ifdef DEBUG
#undef DEBUG
#endif
//#define DEBUG

// input
const int ligPinLft = 1;    // gpio
const int ligPinRgt = 2;    // gpio

// output
Servo servoDir;    // create servo object to control a servo
const int servoPinDir = 5;    // servo(pwm)     Timer1A
const int motorPinDir = 3;    // gpio
const int motorPinPwm = 6;    // pwm            Timer1B

// periodic task
void periodic();    // Timer2

// Rq. Timer0 is used for millis/micros

void setup() {
#ifdef DEBUG
    // debug
    Serial.begin(19200);
    pinMode(13, OUTPUT);
#endif

    // input
    pinMode(ligPinLft, INPUT);
    pinMode(ligPinRgt, INPUT);

    // output
    pinMode(motorPinDir, OUTPUT);
    servoDir.attach(servoPinDir);    // attaches the servo on pin servoPinDir to the servo object

    // periodic task
    MsTimer2::set(10, periodic);    // 10ms periodic task
    MsTimer2::start();
}

// input
//    analogRead(analogPin)==0 -> 1023
//    digitalRead(gpioPin)==LOW | HIGH

// output
//    digitalWrite(gpioPin, LOW | HIGH);
//    servoObj.write(0 -> 180);
//    analogWrite(pwmPin, 0 -> 255);

#define CLAMP(m, n, M) min(max((m), (n)), (M))

#ifdef DEBUG
volatile unsigned int update=0;
unsigned int nb_calc=0;

bool _ligLft, _ligRgt;
int _angle;
unsigned int update=0, _pos_c, _nb_calc;
#endif

void periodic() {
    int speed=50, angle=0;
    bool ligLft, ligRgt;
    ePosition pos_t;
    static ePosition pos_c, pos_p=pLOST;

// read sensors data (HIGH => on the line, LOW => on the blue)
    ligLft = digitalRead(ligPinLft);
    ligRgt = digitalRead(ligPinRgt);

// position estimation
    pos_t=pos_c;
    pos_c=(ePosition)pos_next[POS(pos_p)][ligLft][ligRgt];
    pos_p=pos_t;

    switch(POS(pos_c)) {
    case pLOST:
                        angle = 0;      speed = 50;
        break;
    case pLEFT:
                        angle = -45;    speed = 70;
        break;
    case pLEFT_CENTER:
                        angle = -20;    speed = 90;
        break;
    case pCENTER:
                        angle = 0;      speed = 90;
        break;
    case pRIGHT_CENTER:
                        angle = +20;    speed = 90;
        break;
    case pRIGHT:
                        angle = +45;    speed = 70;
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
    nb_calc++;

    if(!update) {
        // copy the variables to be sure we have consistent data from the same run
        _ligLft=ligLft;
        _ligRgt=ligRgt;
        _angle=angle;
        _pos_c=pos_c;
        _nb_calc=nb_calc;

        update=1;   // tell main task the data has been updated
    }
#endif
}

#ifdef DEBUG
long tempo_led=0;
int etat_led=0;
#endif

void loop() {
#ifdef DEBUG
    if(millis()-tempo_led>100) {
        tempo_led=millis();
        etat_led^=1;
        digitalWrite(13, etat_led);
    }

    if(update) {
        // print data
        Serial.print(millis()); // t
        Serial.print(",");
        Serial.print(_nb_calc); // t
        Serial.print(",");
        Serial.print((unsigned int)_pos_c);
        Serial.print(",");
        Serial.print(_ligLft);
        Serial.print(",");
        Serial.print(_ligRgt);
        Serial.print(",");
        Serial.print(_angle);
        Serial.print("\r\n");

        // acknowledge
        update=0;
    }
#endif
}

