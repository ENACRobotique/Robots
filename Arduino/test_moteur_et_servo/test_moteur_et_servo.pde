#include <Servo.h> 

Servo servoDir;  // create servo object to control a servo 

// input
int potPinDir = A0; // analog
int butPinMot = 2; // gpio

// output
int motorPinPwm = 3;  // pwm
int motorPinDir = 4;  // gpio
int servoPinDir = 5;  // servo(pwm)

void setup()  {
  // debug
  Serial.begin(9600); 

  // input
  pinMode(butPinMot, INPUT);

  // output
  pinMode(motorPinDir, OUTPUT);
  servoDir.attach(servoPinDir);  // attaches the servo on pin 9 to the servo object
}

// input
//  val = analogRead(analogPin);            // reads the value of the potentiometer (value between 0 and 1023) 
//  digitalRead(gpioPin)==LOW | HIGH

// output
//  digitalWrite(gpioPin, LOW | HIGH);
//  servoDir.write(0 -> 180);
//  analogWrite(pwmPin, 0 -> 255);

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

