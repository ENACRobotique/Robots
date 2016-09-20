/*
 * smarties_sorter_color_reco.h
 *
 *  Created on: 20 sept. 2016
 *      Author: buisangu
 */

#ifndef LIBRAIRIES_SMARTIES_SORTER_COLOR_RECO_H_
#define LIBRAIRIES_SMARTIES_SORTER_COLOR_RECO_H_

#define PIN_RED A0
#define PIN_GREEN A1
#define PIN_BLUE A2

typedef enum{
	FAIL, //This is the default choice, DO NOT ERASE !
	PINK,
	PURPLE,
	BLUE,
	GREEN,
	YELLOW,
	ORANGE,
	RED,
	BROWN
}eSmartiesColor;

typedef struct{
	eSmartiesColor color;
	int red_min;
	int red_max;
	int green_min;
	int green_max;
	int blue_min;
	int blue_max;
}sSmartiesData;

typedef struct{
	int red;
	int green;
	int blue;
}sRawColorValues;

sSmartiesData colorTable[] = {
	//	COLOR, R_MIN, R_MAX, G_MIN, G_MAX, B_MIN, B_MAX
		{PINK, 200, 255, 100, 125, 50, 75},
		{PURPLE, 200, 255, 100, 125, 200, 250},
		{BLUE, 50, 75, 75, 100, 200, 255},
};

#define COLOR_TABLE_SIZE (sizeof(colorTable)/sizeof(*colorTable))

extern void initSmartiesReco();
extern eSmartiesColor readColor();
extern sRawColorValues readRawData();

#endif /* LIBRAIRIES_SMARTIES_SORTER_COLOR_RECO_H_ */
