/*
 * main.cpp
 *
 *  Created on: dec. 2015
 *      Author: Fabien
 */
#include "Arduino.h"
#include "SoftwareServo.h"
#include "EEPROM.h"

#define SERVO 1
#define BOUTON 0
#define LED1 3
#define LED2 4

#define DELAY_LED_BLINKING 800

#define TIME_BLINK 5000
SoftwareServo servo;
unsigned int nbCoups;

unsigned int compteurCentaines;
unsigned int compteurDizaines;

void setup()
{
	compteurCentaines = compteurDizaines = 0;
	nbCoups = 0;
	unsigned char tmp = EEPROM.read(0); //adresse = 0 : poids fort
	nbCoups = tmp << 8;
	tmp = EEPROM.read(1); //adresse = 1 : poids faible
	nbCoups |= tmp;

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
		compteurCentaines = 0;
		compteurDizaines = 0;
//		digitalWrite(LED1,HIGH);
//		digitalWrite(LED2,HIGH);
		PORTB |= ((1<<LED1) | (1<<LED2));
		servo.write(120);
		if(!activated) {
			nbCoups++;
			unsigned char tmp = nbCoups & 0xFF;
			EEPROM.write(1,tmp);
			tmp = nbCoups >> 8;
			EEPROM.write(0,tmp);
		}
		activated = 1;
		last_time_activity = millis();

		EEPROM.write(124,42);
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

	if((millis() - last_time_activity > TIME_BLINK) && (millis() - time_led > DELAY_LED_BLINKING))	//au bout de 15s d'inactivité, on fait clignoter les leds
	{
		unsigned int cents = nbCoups/100;

		if(compteurCentaines < 2*cents ) {
			PORTB ^= 1<<LED1;
			compteurCentaines++;
		}
		else {
			if(compteurDizaines < 2 * ((nbCoups - 100*cents )/10) ) {
				PORTB ^= 1<<LED2;
				compteurDizaines++;
			}
		}

//		led_state ^= 1;
//		digitalWrite(LED1, led_state);
//		digitalWrite(LED2, led_state);
		//PORTB ^= ((1<<LED1) | (1<<LED2));
		time_led = millis();
	}
}
