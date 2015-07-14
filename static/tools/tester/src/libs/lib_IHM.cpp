/*
 * state-blink.cpp
 *
 *  Created on: 15 mai 2013
 *      Author: quentin
 */

#include <Arduino.h>
#include <params.h>
//#include <state_types.h>
#include <stddef.h>
#include <stdarg.h>
#include <tools.h>
#include "lib_IHM.h"

#ifndef NOLCD
LiquidCrystal lcd(LCD1,LCD2,LCD3,LCD4,LCD5,LCD6);
#endif

Encoder myEnc(ENCODER1, ENCODER2);


void afficher(int col, int row, const char * format...)
{
	char buffer[17];

	va_list ap;
	va_start(ap, format);
	vsnprintf(buffer, sizeof(buffer), format, ap);
	va_end(ap);

#ifndef NOLCD
	eraseLine(row);
	lcd.setCursor(col, row);
	lcd.write(buffer);
#endif
#ifdef NOLCD
		Serial.println(buffer);
#endif
}

void afficher(const char * format...)
{
	lcd.clear();
	//afficher(0,0,format);
	char buffer[17];

	va_list ap;
	va_start(ap, format);
	vsnprintf(buffer, sizeof(buffer), format, ap);
	va_end(ap);

#ifndef NOLCD
	lcd.home();
	lcd.write(buffer);
#endif
#ifdef NOLCD
		Serial.println(buffer);
#endif
}

void eraseLine(int row)	//erase the line you specify (0 or 1 in two line display)
{
	lcd.setCursor(0, row);
	lcd.write("                ");
}

void startLcd()
{
	lcd.begin(16,2);
}

void clearLcd(){
	lcd.clear();
}
