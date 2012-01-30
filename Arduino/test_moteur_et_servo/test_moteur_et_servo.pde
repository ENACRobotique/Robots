#include <Servo.h>

// input
const int butPinMot = 2; // gpio
const int potPinDir = A0; // analog

// output
Servo servoDir;  // create servo object to control a servo
const int servoPinDir = 5;  // servo(pwm)     // Timer1A
const int motorPinDir = 4;  // gpio
const int motorPinPwm = 6;  // pwm            // Timer1B

// Rq. Timer0 is used for millis/micros

void setup()  {
  // debug
  Serial.begin(9600);

  // input
  pinMode(butPinMot, INPUT);

  // output
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

void loop()  {
  // direction
  int dir=analogRead(potPinDir);

  dir = map(dir, 0, 1023, 0, 180);

  Serial.print("dir = " );
  Serial.print(dir);
  Serial.print("\r\n");

  servoDir.write(dir);                  // sets the servo position according to the scaled value

  // vitesse
  digitalWrite(motorPinDir, HIGH);  // go forward
  if(digitalRead(butPinMot)==HIGH)
    analogWrite(motorPinPwm, 127);  // half speed
  else
    analogWrite(motorPinPwm, 0);  // stop

  delay(20);
}

