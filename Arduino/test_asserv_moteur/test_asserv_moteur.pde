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
//    analogRead(analogPin)==0 -> 1023
//    digitalRead(gpioPin)==LOW | HIGH

// output
//    digitalWrite(gpioPin, LOW | HIGH);
//    servoDir.write(0 -> 180);
//    analogWrite(pwmPin, 0 -> 255);

#define CLAMP(m, n, M) min(max((m), (n)), (M))

uint16_t _raw2dist[] = {
#if 0
//    <<7
    21367, 14718, 11208, 9038, 7564, 6497, 5689, 5056,
    4547, 4128, 3778, 3481, 3225, 3003, 2809, 2636,
    2483, 2346, 2222, 2110, 2008, 1915, 1830, 1751,
    1678, 1611, 1548, 1490, 1435, 1384, 1336, 1292,
    1249, 1209, 1172, 1136, 1102, 1070, 1039, 1010,
    983, 956, 931, 907, 884, 862, 841, 821,
    801, 782, 765, 747, 731, 714, 699, 684,
    670, 656, 642, 629, 616, 604, 592, 581,
    570, 559, 548, 538, 528, 519, 509, 500,
    491, 483, 474, 466, 458, 450, 443, 435,
    428, 421, 414, 408, 401, 395, 388
#else
//    <<4
    2671, 1840, 1401, 1130, 945, 812, 711, 632,
    568, 516, 472, 435, 403, 375, 351, 330,
    310, 293, 278, 264, 251, 239, 229, 219,
    210, 201, 194, 186, 179, 173, 167, 161,
    156, 151, 146, 142, 138, 134, 130, 126,
    123, 120, 116, 113, 111, 108, 105, 103,
    100, 98, 96, 93, 91, 89, 87, 85,
    84, 82, 80, 79, 77, 76, 74, 73,
    71, 70, 69, 67, 66, 65, 64, 63,
    61, 60, 59, 58, 57, 56, 55, 54,
    54, 53, 52, 51, 50, 49, 49
#endif
};


inline int raw2dist(int m) {    // returns centimeters<<4
    // m is from 0 to 1023

    // yes, it should be "7-(m&7)" instead of "8-(m&7)" but then, it would be "/7" instead of ">>3" and it sucks
    return ( _raw2dist[m>>3]*(8-(m&7)) + _raw2dist[(m>>3)+1]*(m&7) )>>3;
}

void loop() {
    // distance
    int dist=analogRead(optPinDis);
    static int I=0;
    long tmp;

Serial.print("dist = ");
Serial.print(dist);
Serial.print("\t, ");
    dist=raw2dist(dist);
Serial.print((float)dist/16.0);
Serial.print("\r\n");

    tmp = dist-(20<<4);  // epsilon
    I = CLAMP(-(64<<4), I+tmp, 64<<4);

Serial.print("I = ");
Serial.print((float)I/16.0);
Serial.print("\r\n");

    tmp = -4*(tmp + 0*(I>>5));   // Kp*(epsilon + Ki*I) (>>5 is dt)

Serial.print("s = ");
Serial.print((tmp>>4)+47);
Serial.print("\r\n");

    servoDir.write(CLAMP(0, (tmp>>4)+47, 47*2));    // sets the servo position (47 is ~neutral)

    // vitesse
    digitalWrite(motorPinDir, HIGH);    // direction: forward
    if(digitalRead(butPinMot)==HIGH)
        analogWrite(motorPinPwm, 100);    // go
    else
        analogWrite(motorPinPwm, 0);    // stop

    delay(30);  // 33,3...Hz
}

