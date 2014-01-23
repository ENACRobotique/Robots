/*
 * lib_radar2.cpp
 *
 *  Created on: 5 déc. 2013
 *      Author: seb
 */

#include "lib_radar2.h"


//globals
uint16_t C_rad[ RAD_NB_PTS ]; //last range values read
uint16_t C_rad_limit[ RAD_NB_PTS ]; //limit values for each sensing direction, under which the robot shall stop (opponent avoidance)
/* how are stored the values in C_rad and C_rad_limits (0° pointing toward the front of the robot, angles increasing clockwise):
index  angle  direction
0      202,5  rear-left
1      157,5  rear-right
2      112,5  right-rear
3      67,5   right-front
4      22,5   front-right
5      337,5  front-left
6      292,5  left-front
7      247,5  left-rear*/
///!\ valid for min=65 && max=110

void radarSetLim(uint16_t limits[RAD_NB_PTS])
	{
    memcpy(&C_rad_limit[RAD_NB_PTS], &limits[RAD_NB_PTS],sizeof(uint16_t)*RAD_NB_PTS );
	#ifdef DEBUG_RADAR
		Serial.println("Radar limite actuel :");
		Serial.println(C_rad_limit[0]);
		Serial.println(C_rad_limit[1]);
	#endif
	}


//function to call periodically to refresh the values read by the radar
void radarRefresh()
	{
	unsigned long time=millis();
	static unsigned long time_prev_rad=0;
	static int etat_rad = 0;

	if(etat_rad)
		{
		if((time-time_prev_rad)>=RAD_TIMER_1)
			{
			if ((time-time_prev_rad)>=RAD_TIMER_1+RAD_TIMER_1/2) time_prev_rad=time-RAD_TIMER_1; //to avoid problems due to long loop
			time_prev_rad = time_prev_rad + RAD_TIMER_1;

			// get the 2 ranges
			for(int nb = 0; nb<2; nb++)
				{
				C_rad[nb] = getRangeResult(nb);
				}
			etat_rad = 0;

			#ifdef DEBUG_RADAR
				int i;
				for (i=0;i<RAD_NB_PTS;i++)
					{
					Serial.print(C_rad[i]);
					Serial.print("|");
					Serial.print(C_rad_limit[i]);
					Serial.print(",\t");
					}
				Serial.println(" ");
			#endif
			}
		}
	else
		{
		if((time-time_prev_rad)>=RAD_TIMER_1)
			{
			if ((time-time_prev_rad)>=(RAD_TIMER_1)/2) time_prev_rad=time; //to avoid problems due to long loop

		  // start the 2 ranges
		  for(int nb = 0; nb<2; nb++)
		  	  {
			  startRange(nb);
		  	  }
		  etat_rad = 1;
			}
		}
	}

//returns the shortest range measured

uint16_t radarCloser()
	{
	uint16_t mini=999;
	int i;
	for (i=0;i<RAD_NB_PTS;i++)
		{
		if(C_rad[i]<mini) mini=C_rad[i];
		}
  return mini;
}

//returns the number of range measures shorter than the limit defined in C_rad_limits
//(for the direction associated to each measure).
int radarIntrusion()
	{
	int nb=0;
	int i;
	for (i=0;i<RAD_NB_PTS ;i++)
		{
		if( (C_rad[i]<C_rad_limit[i]) && C_rad[i]) nb++; //Cas limite
		}
	return nb;
	}


