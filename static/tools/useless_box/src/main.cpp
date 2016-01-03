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

#define TIME_BLINK 15000
SoftwareServo servo;

void setup()
{
//	pinMode(BOUTON,INPUT_PULLUP);
//	pinMode(LED1,OUTPUT);
//	pinMode(LED2,OUTPUT);
	DDRB = 	0 | (1<<LED1) | (1<<LED2); 	// les leds sont mises en sortie
	PORTB = 0 | (1<<BOUTON);			//le bouton en pull-up
	servo.attach(SERVO);
	servo.setMaximumPulse(2200);
	servo.write(0);
}

unsigned long tps_servo = 0;
unsigned long last_time_activity = 0;
unsigned long time_led = 0;
char activated = 0;
//int led_state = 0;

void loop()
{
	if(digitalRead(BOUTON))
	{
//		digitalWrite(LED1,HIGH);
//		digitalWrite(LED2,HIGH);
		PORTB |= ((1<<LED1) | (1<<LED2));
		servo.write(120);
		activated = 1;
		last_time_activity = millis();
	}
	else
	{
		if(activated)
		{
//			digitalWrite(LED1,LOW);
//			digitalWrite(LED2,LOW);
			PORTB &= ~((1<<LED1) | (1<<LED2));
			servo.write(0);
			activated = 0;
		}
	}


	if(millis()-tps_servo > 20)
	{
		SoftwareServo::refresh();
		tps_servo = millis();
	}

	if((millis() - last_time_activity > TIME_BLINK) && (millis() - time_led > 800))	//au bout de 15s d'inactivité, on fait clignoter les leds
	{
//		led_state ^= 1;
//		digitalWrite(LED1, led_state);
//		digitalWrite(LED2, led_state);
		PORTB ^= ((1<<LED1) | (1<<LED2));
		time_led = millis();
	}
}
