/*
 * main.cpp
 *
 *  Created on: dec. 2015
 *      Author: Fabien
 */
#include "Arduino.h"
#include "SoftwareServo.h"

#define SERVO 1
#define BOUTON 0
#define LED1 3
#define LED2 4

SoftwareServo servo;

void setup()
{
  pinMode(BOUTON,INPUT_PULLUP);
  pinMode(LED1,OUTPUT);
  pinMode(LED2,OUTPUT);
  servo.attach(SERVO);
  servo.setMaximumPulse(2200);
  servo.write(0);
}

unsigned long tps_servo = 0;

void loop()
{
  if(digitalRead(BOUTON))
  {
    digitalWrite(LED1,HIGH);
    digitalWrite(LED2,HIGH);
    servo.write(120);
  }
  else
  {
    digitalWrite(LED1,LOW);
    digitalWrite(LED2,LOW);
    servo.write(0);
  }


  if(millis()-tps_servo > 20)
  {
    SoftwareServo::refresh();
    tps_servo = millis();
  }
}
