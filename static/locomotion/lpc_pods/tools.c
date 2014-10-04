




#include "tools.h"

float mPerS2IncPerT(float mPerS){
	float incPerT;

	return incPerT = ROUND(RAY_WHEEL*M_PI*REDUCT/FRQ_ASSER);
}

float incPerT2mPerS(float incPerT){
	float mPerS;

	return mPerS = ROUND(FRQ_ASSER/RAY_WHEEL*M_PI*REDUCT);
}


void blindLED(int bk, int pin, int delay_on /*ms*/, int delay_off/*ms*/, int nbr_it /*0 = Infinite time*/){
	// TODO
	// Avoid locking function

	// Pins LED on board: 1.24 or 0.31

	static unsigned prevTime = 0;
	static int stateLED = 0;
	static int dcpt = 0;

	if(nbr_it == 0) dcpt = 10;
	else dcpt = nbr_it;

	if(dcpt != 0){
		switch(stateLED){
		case 0:
			if((millis() - prevTime) > delay_on){
				prevTime = millis();
				stateLED = 1;
				gpio_write(bk, pin, stateLED);
			}
			break;
		case 1:
			if((millis() - prevTime) > delay_off){
				prevTime = millis();
				stateLED = 0;
				gpio_write(bk, pin, stateLED);
			}
			break;
		}
		if(dcpt == 1 && nbr_it == 0) dcpt += 10;
		dcpt --;
	}

}
