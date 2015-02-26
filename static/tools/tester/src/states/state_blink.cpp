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
#include <tools.h>
#include "state_blink.h"

#ifndef NOLCD
#include "../../../../core/arduino/libraries/LiquidCrystal/LiquidCrystal.h"
LiquidCrystal lcd(LCD1,LCD2,LCD3,LCD4,LCD5,LCD6); //pins a vérifier
#endif


void afficher(const char * chaine)
{
#ifndef NOLCD
	  lcd.clear();
	  lcd.home();
	  lcd.write(chaine);
#endif
#ifdef NOLCD
		Serial.println(chaine);
#endif
}

