#include <Servo.h>  // shipped with arduino-core
#include <MsTimer2.h>   // get this library at http://arduino.cc/playground/Main/MsTimer2

#include "sharp_2d120x.h"

// input
const int butPinMot = 2; // gpio
const int optPinDis = A1;    // analog

// output
Servo servoDir;    // create servo object to control a servo
const int servoPinDir = 5;    // servo(pwm)     Timer1A
const int motorPinDir = 4;    // gpio
const int motorPinPwm = 6;    // pwm            Timer1B

// periodic task
void periodic();    // Timer2

// Rq. Timer0 is used for millis/micros

void setup() {
    // debug
    Serial.begin(9600);

    // input
    pinMode(butPinMot, INPUT);

    // output
    pinMode(motorPinDir, OUTPUT);
    servoDir.attach(servoPinDir);    // attaches the servo on pin servoPinDir to the servo object

    // periodic task
    MsTimer2::set(30, periodic);    // 30ms periodic task
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

int update=0;

unsigned int dist;
unsigned int last_dist=0;
int I=0;
int tmp, D, eps;

#define DTAB_BITS 2
unsigned int dtab_i=0, dtab_sum=0;
unsigned int dtab[1<<DTAB_BITS]={0};

void periodic() {
// get distance from Sharp sensor
    // this is just a mean to avoid the noises/spikes on the sensor data (FIXME: use a kalman filter?)
    dtab_sum-=dtab[dtab_i];
    dtab[dtab_i]=analogRead(optPinDis);
    dtab_sum+=dtab[dtab_i];
    dtab_i=(dtab_i+1)&((1<<DTAB_BITS)-1);

    dist=dtab_sum>>DTAB_BITS;   // get mean of all the terms of the array

    dist=raw2dist(dist);    // get distance in centimeters<<4

// compute epsilon
    tmp = dist-(20<<4); // the set point is 20 centimeters
    eps=tmp;

// compute integral term by successive sums
    I = CLAMP(-(64<<4), I+tmp, 64<<4);  // clamp the integral term to limit overrun (TODO: find good limits)

// compute derivative term
    D = (dist - last_dist)<<5;  // <<5 is ~1/dt (dt is ~30ms)
    last_dist = dist;

// Ziegler-Nichols: Ku~=18, Tu~=1s

//    tmp = -11*tmp + -((22*I)>>5) + -2*D;  // PID: Kp*epsilon + Ki*I + Kd*D (>>5 is ~dt)
//    tmp = -8*tmp + -((10*I)>>5);          // PI:  Kp*epsilon + Ki*I (>>5 is ~dt)
    tmp = -9*tmp;                           // P:   Kp*epsilon

    servoDir.write(CLAMP(0, (tmp>>4)+47, 47*2));    // sets the servo position (47 is ~neutral)

// set speed
    digitalWrite(motorPinDir, HIGH);    // direction: forward
    if(digitalRead(butPinMot)==HIGH)
        analogWrite(motorPinPwm, 160);    // go
    else
        analogWrite(motorPinPwm, 0);    // stop

    update=1;   // tell main task the data has been updated
}

void loop() {
    static int times = 0;

    if(update) {
        // to be sure we have consistent data from the same run
        unsigned int _dist=dist;
        int _I=I;
        long _tmp=tmp, _D=D, _eps=eps;

        if(--times<=0) {
            Serial.print("dist\teps\tI\tD\ts\r\n");
            times=20;
        }

        Serial.print((float)_dist/16.0); // dist
        Serial.print("\t");
        Serial.print((float)_eps/16.0); // eps
        Serial.print("\t");
        Serial.print((float)_I/16.0);    // I
        Serial.print("\t");
        Serial.print((float)_D/16.0);    // D
        Serial.print("\t");
        Serial.print((_tmp>>4)+47);      // s
        Serial.print("\r\n");

        update=0;
    }
}

