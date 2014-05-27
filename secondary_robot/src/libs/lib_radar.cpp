/*
Library to use with the "ultrasonic radar" of the secondary robot of the ENAC for the competition of 2013.
Rely on the library for the SRF02 sensors (and the latter sensors).
Simple version, with only 2 positions (we cover the whole surroundings of the robot within a range of 1,2m with SRF02 sensorss)

*/

/*#include "lib_radar.h"


//globals
Servo servoRad; // servo object for the servomotor handling the radar rotation
uint16_t C_rad[ RAD_NB_PTS ]; //last range values read
uint16_t C_rad_limit[ RAD_NB_PTS ]; //limit values for each sensing direction, under which the robot shall stop (opponent avoidance)

/* how are stored the values in C_rad and C_rad_limits (0° pointing toward the front of the robot, angles increasing clockwise):
old index  angle  direction
old 0      202,5  rear-left
old 1      157,5  rear-right
old 2      112,5  right-rear
old 3      67,5   right-front
old 4      22,5   front-right
old 5      337,5  front-left
old 6      292,5  left-front
old 7      247,5  left-rear/
///!\ valid for min=65 && max=110 */

void radarSetLim(uint16_t limits[RAD_NB_PTS]){
    memcpy(C_rad_limit,limits,sizeof(uint16_t)*RAD_NB_PTS);
}

//initialises the pin for the servo, the servo and its first position.
//REQUIRES : Wire.begin()
// only useful when a servo is used,  do not call otherwise
void radarInitServo(int pinRadarServo){
    pinMode(pinRadarServo,OUTPUT);
	servoRad.attach(pinRadarServo);
	servoRad.write(RAD_POS_MIN);
}

//function to call periodically to refresh the values red by the radar
void radarRefresh(){
 unsigned long time=millis();
 static unsigned long time_prev_rad=0;  
 static int pos_rad = RAD_POS_MIN, etat_rad = 0,sens_rad=1;
  // gestion radar
if(etat_rad) {
    if((time-time_prev_rad)>=RAD_TIMER_1) {
        if ((time-time_prev_rad)>=RAD_TIMER_1+RAD_TIMER_1/2) time_prev_rad=time-RAD_TIMER_1; //to avoid problems due to long loop
        time_prev_rad = time_prev_rad + RAD_TIMER_1;

        // get the ranges
        for(int nb = 0; nb<RAD_NB_SENSORS; nb++) {
        C_rad[ (pos_rad-RAD_POS_MIN)/RAD_POS_INC + (RAD_NB_SENSORS-1-nb)*RAD_NB_POS] = getRangeResult(nb);
        }
        if(servoRad.attached()){
			//move the servo
			if(sens_rad && pos_rad < RAD_POS_MAX)
				{
				 pos_rad = pos_rad + RAD_POS_INC;
				 if(pos_rad == RAD_POS_MAX) sens_rad = 0;
				}
			else if(!sens_rad && pos_rad > RAD_POS_MIN)
			{
			  pos_rad = pos_rad - RAD_POS_INC;
			  if(pos_rad == RAD_POS_MIN) sens_rad = 1;
			}
        	servoRad.write(pos_rad);
        }


        etat_rad = 0;
#ifdef DEBUG_RADAR
int i;
for (i=0;i<RAD_NB_PTS;i++) {
  Serial.print(C_rad[i]);
  Serial.print("|");
  Serial.print(C_rad_limit[i]);
  Serial.print(",\t");
}
Serial.println(" ");
#endif
    }
  }
  else {
    if((time-time_prev_rad)>=RAD_TIMER_2) {
      if ((time-time_prev_rad)>=RAD_TIMER_2+(RAD_TIMER_2)/2) time_prev_rad=time-RAD_TIMER_2; //to avoid problems due to long loop
      time_prev_rad = time_prev_rad + RAD_TIMER_2;
 
      // start the ranges
      for(int nb = 0; nb<RAD_NB_SENSORS; nb++) {
        startRange(nb);
      }
      etat_rad = 1;
    }
  } 
}

//returns the shortest range measured
uint16_t radarCloser(){
  uint16_t mini=999;
  int i;
  for (i=0;i<RAD_NB_PTS;i++){
    if(C_rad[i]<999) mini=C_rad[i];
  }
  return mini;
}

//returns the number of range measures shorter than the limit defined in C_rad_limits 
//(for the direction associated to each measure).
int radarIntrusion(){
  int nb=0;
  int i;
  for (i=0;i<RAD_NB_PTS ;i++){
    if(C_rad[i]<C_rad_limit[i] && C_rad[i]) nb++;
  }
  return nb; 
}

