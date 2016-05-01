#include "Encoder.h"
#include "Arduino.h"

#define DEBOUNCE_DELAY 10
#define BIG_STEP 25

Encoder::Encoder(int p1, int p2) : pin1(p1), pin2(p2)
{
	position=0;
	min=-32768;
	max=32767;
	prev=millis();
	base_inc = 1;
	fast_inc = 1;
	init();
}

int Encoder::read()
{
	return position;
}

void Encoder::write(int nb)
{
	position = CLAMP(min,nb,max);
}

void Encoder::init()
{
	pinMode(pin1, INPUT_PULLUP);
	pinMode(pin2, INPUT_PULLUP);
	//do attachInterrupt(pin1-2,myenc.update,CHANGE);
}

void Encoder::setLimits(int mini, int maxi)
{
	min = mini;
	max = maxi;
	position = CLAMP(min,position,max);
}

void Encoder::update()
{
	if (millis()-prev > DEBOUNCE_DELAY){
		int mult= base_inc;
		if(millis()-prev < BIG_STEP) mult = fast_inc;
		prev = millis();
		if (digitalRead(pin1)) position -= ((digitalRead(pin2)<<1) -1)*mult;
		else position += ((digitalRead(pin2)<<1) -1)*mult;
		write(position);
	}
}

void Encoder::setMultiplicators(int base_multiplicator, int fast_multiplicator)
{
	base_inc = base_multiplicator;
	fast_inc = fast_multiplicator;
}
