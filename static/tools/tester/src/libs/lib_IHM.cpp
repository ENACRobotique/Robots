/*
 * state-blink.cpp
 *
 *  Created on: 15 mai 2013
 *      Author: quentin
 */

#include <Arduino.h>
#include <params.h>
#include <state_types.h>
#include <stddef.h>
#include <stdarg.h>
#include <tools.h>
#include "lib_IHM.h"

#ifndef NOLCD
#include "../../../../core/arduino/libraries/LiquidCrystal/LiquidCrystal.h"
LiquidCrystal lcd(LCD1,LCD2,LCD3,LCD4,LCD5,LCD6); //pins a vérifier
#endif

Encoder myEnc(ENCODER1, ENCODER2);

void afficher(const char * format...)
{
	char buffer[17];

	va_list ap;
	va_start(ap, format);
	vsnprintf(buffer, sizeof(buffer), format, ap);
	va_end(ap);

#ifndef NOLCD
	  lcd.clear();
	  lcd.home();
	  lcd.write(buffer);
#endif
#ifdef NOLCD
		Serial.println(buffer);
#endif
}

