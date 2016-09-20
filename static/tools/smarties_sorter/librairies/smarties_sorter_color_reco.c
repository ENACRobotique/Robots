/*
 * smarties_sorter_color_reco.c
 *
 *  Created on: 20 sept. 2016
 *      Author: buisangu
 */

#include "smarties_sorter_color_reco.h"
#include "Arduino.h"

void initSmartiesReco(){
	pinMode(PIN_RED, INPUT);
	pinMode(PIN_GREEN, INPUT);
	pinMode(PIN_BLUE, INPUT);
}

eSmartiesColor readColor(){
	sRawColorValues colorsRead = readRawData();
	int red = colorsRead.red, green = colorsRead.green, blue = colorsRead.blue;
	int i;
	for (i = 0; i < COLOR_TABLE_SIZE; i++){
		sSmartiesData smartiesData = colorTable[i];
		if ((red >= smartiesData.red_min && red <= smartiesData.red_max) && (green >= smartiesData.green_min && green <= smartiesData.green_max)
				&& (blue >= smartiesData.blue_min && blue <= smartiesData.blue_max)){
			return smartiesData.color;
		}
	}
	return FAIL;
}

sRawColorValues readRawData(){
	sRawColorValues ret = {analogRead(PIN_RED), analogRead(PIN_GREEN), analogRead(PIN_BLUE)};
	return ret;
}
