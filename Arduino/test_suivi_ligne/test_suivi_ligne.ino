#include <Servo.h>  // shipped with arduino-core
#include <MsTimer2.h>   // get this library at http://arduino.cc/playground/Main/MsTimer2

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
    Serial.begin(9600);
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
int update=0;
#endif

bool ligLft, ligRgt;

void periodic() {
// get line position from sensors
    int speed=100, angle=0;

    ligLft = digitalRead(ligPinLft);
    ligRgt = digitalRead(ligPinRgt);

    if(ligLft && ligRgt)
        speed = 50;
    else if(ligLft)
        angle = +15;
    else if(ligRgt)
        angle = -15;

    servoDir.write(CLAMP(0, angle+45, 45*2));    // sets the servo position (45 is ~neutral)

// set speed
    digitalWrite(motorPinDir, HIGH);    // direction: forward
    analogWrite(motorPinPwm, speed);    // go

#ifdef DEBUG
    update=1;   // tell main task the data has been updated
#endif
}

void loop() {
#ifdef DEBUG
    if(update) {
        // copy the variables to be sure we have consistent data from the same run

#if 0   // pretty print data
        static int times = 0;

        if(--times<=0) {
            Serial.print("dist\teps\tI\tD\ts\r\n");
            times=20;
        }

#else   // print data (don't miss any measurement)
        Serial.print(millis()); // t
//        Serial.print(",");
        Serial.print("\r\n");
#endif

        update=0;
    }
#endif
}

