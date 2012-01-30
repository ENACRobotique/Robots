#include <WProgram.h>
#include <Servo.h>

Servo servoDir;    // create servo object to control a servo

// input
int butPinMot = 2; // gpio
int optPinDis = A1;    // analog

// output
int motorPinPwm = 3;    // pwm
int motorPinDir = 4;    // gpio
int servoPinDir = 5;    // servo(pwm)

void setup() {
    // debug
    Serial.begin(9600);

    // input
    pinMode(butPinMot, INPUT);

    // output
    pinMode(motorPinDir, OUTPUT);
    servoDir.attach(servoPinDir);    // attaches the servo on pin 9 to the servo object
}

// input
//    val = analogRead(analogPin);                        // reads the value of the potentiometer (value between 0 and 1023)
//    digitalRead(gpioPin)==LOW | HIGH

// output
//    digitalWrite(gpioPin, LOW | HIGH);
//    servoDir.write(0 -> 180);
//    analogWrite(pwmPin, 0 -> 255);

#define CLAMP(m, n, M) min(max(m, n), M)

inline int raw2dist(int m) {    // TODO: use table and return cm*8
    // m is from 0 to 1023

    return 24080/(143+(m<<3))-1;    // return distance in cm
}

void loop() {
    // distance
    int tmp, dist=analogRead(optPinDis);
    static int I=0;

Serial.print("dist = ");
Serial.print(dist);
Serial.print("\t, ");
    dist=raw2dist(dist);
Serial.print(dist);
Serial.print("\r\n");

    tmp = dist-20;  // epsilon
    I = CLAMP(-64, I+tmp, 64);

Serial.print("I = ");
Serial.print(I);
Serial.print("\r\n");

    tmp = -16*(tmp + 0*(I>>5));   // Kp*(epsilon + Ki*I)

Serial.print("s = ");
Serial.print(tmp+47);
Serial.print("\r\n");

    servoDir.write(CLAMP(0, tmp+47, 47*2));    // sets the servo position (47 is ~neutral)

    // vitesse
    digitalWrite(motorPinDir, HIGH);    // direction: forward
    if(digitalRead(butPinMot)==HIGH)
        analogWrite(motorPinPwm, 100);    // go
    else
        analogWrite(motorPinPwm, 0);    // stop

    delay(30);  // 33,3...Hz
}

