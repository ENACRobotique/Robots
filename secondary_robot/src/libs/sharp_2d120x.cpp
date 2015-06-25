#include "sharp_2d120x.h"
#include "Arduino.h"
extern "C" {
#include "median_filter.h"
}
#include "lib_attitude.h"

#define PERIOD_UPDATE 40

int C_sharp_limit[ NB_SHARP ];
int pin_sharp[NB_SHARP];
median_t mf_dist[NB_SHARP];

unsigned int _raw2dist120x[] = {    // sizeof(unsigned int)==2 => 65535 max
//  _raw2dist120x[raw>>3] is in centimers<<4 when raw is from 0 to 1023
    2671, 1840, 1401, 1130, 945, 812, 711, 632,
    568, 516, 472, 435, 403, 375, 351, 330,
    310, 293, 278, 264, 251, 239, 229, 219,
    210, 201, 194, 186, 179, 173, 167, 161,
    156, 151, 146, 142, 138, 134, 130, 126,
    123, 120, 116, 113, 111, 108, 105, 103,
    100, 98, 96, 93, 91, 89, 87, 85,
    84, 82, 80, 79, 77, 76, 74, 73,
    71, 70, 69, 67, 66, 65, 64, 63,
    61, 60, 59, 58, 57, 56, 55, 54,
    54, 53, 52, 51, 50, 49, 49
};

int raw2dist120x(int m) {    // returns centimeters<<4
    // m is from 0 to 1023

    // yes, it should be "7-(m&7)" instead of "8-(m&7)" but then, it would be "/7" instead of ">>3" and it sucks
    // the impact on the precision/continuity of the result is negligeable
    return ( _raw2dist120x[m>>3]*(8-(m&7)) + _raw2dist120x[(m>>3)+1]*(m&7) )>>3;
}

int sharpIntrusion(){
  int nb=0;
  int i;
  for (i=0;i<NB_SHARP ;i++){
    if(mf_get(&mf_dist[i])<C_sharp_limit[i] && mf_get(&mf_dist[i])) nb++;
  }
  return nb;
}

void sharpSetLim(int limits[NB_SHARP]){
    memcpy(C_sharp_limit,limits,sizeof(uint16_t)*NB_SHARP);
}

void initSharp(int pinSharp[])
{
	int i;
	for(i=0;i<NB_SHARP;i++){
		mf_init(&mf_dist[i], 5, 0);
		pin_sharp[i] = pinSharp[i];
	}
}

void sharpUpdate()
{
	static unsigned long prev_time = millis();
	if(millis() - prev_time > PERIOD_UPDATE)
	{
		int i;
		for(i=0;i<NB_SHARP;i++){
			int read = analogRead(pin_sharp[i]);
			int distance = raw2dist120x(read) >> 4;
			mf_update(&mf_dist[i], distance);
		}

#ifdef DEBUG_SHARP
		Serial.print("sharp dist :  ");
		for(i=0;i<NB_SHARP;i++){
			Serial.print("mf");
			Serial.print(i);
			Serial.print(": ");
			Serial.print(mf_get(&mf_dist[i]));
			Serial.print("\t");
		}
		Serial.println("");
#endif

	}
}
