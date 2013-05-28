
#include "state_wall.h"
#include "state_pause.h"
#include "state_funny.h"
#include "state_traj.h"

#include "../params.h"
#include "../tools.h"
#include "lib_radar.h"
#include "lib_wall.h"
#include "lib_line.h"
#include "lib_move.h"



unsigned long int saveTime=0;


void initWallLeft(sState *prev) {
    uint16_t tab[RAD_NB_PTS];
    int i;
    for (i=0;i<RAD_NB_PTS;i++){
        tab[i]=15;
    }
    for (i=5;i<=7;i++){
        tab[i]=50;
    }
    radarSetLim(tab);

#ifdef DEBUG
    Serial.println("debut wall left");
#endif
    wallSetVal(LEFT,WALL_DST,WALL_SPEED);

    if (prev==&sPause) {
#ifdef DEBUG
    Serial.println("\tback from pause");
#endif
        saveTime=millis()-saveTime;
    }
    else{
#ifdef UPPERCUT
    armServoLeft.write(ARM_LEFT_DOWN);
#else
    armServoLeft.write(ARM_LEFT_UP);
#endif
    }


}

void deinitWallLeft(sState *next){
    //pause :
    if (next==&sPause) {
        saveTime=millis();
    }
    else {
        saveTime=0;
        move(0,0);
    }

}

sState * testWallLeft(){
    static char nblines=0;
    static unsigned long prevMillisWallLeft;
    static int armPos1,armPos2;
#ifdef UPPERCUT
    armPos1=ARM_LEFT_DOWN;
    armPos2=ARM_LEFT_UP;
#else
    armPos1=ARM_LEFT_UP;
    armPos2=ARM_LEFT_DOWN;
#endif

    static int armPos=armPos1;//ARM_LEFT_UP

    if (armPos==armPos2){
        if ((millis()-prevMillisWallLeft-saveTime)>ARM_RAISE_TIME) {
            armPos=armPos1;
            armServoLeft.write(armPos);
        }
    }
    else if ( getIntensity(4) > LINE_THRESHOLD && getIntensity(3) > LINE_THRESHOLD ) {
        nblines++;
        prevMillisWallLeft=millis();
        saveTime=0;
        armPos=armPos2;
        armServoLeft.write(armPos);
    }

    if ((millis()-_matchStart) > TIME_MATCH_STOP ) return &sFunny;
    if (nblines>=4) {
        move(0,0);
        if ((millis()-prevMillisWallLeft-saveTime)>1000){
            armServoLeft.write(armPos1);
            return &sTrajRedFinal;
        }
    }
    if (radarIntrusion()) return &sPause;
    return 0;
}

sState sWallLeft={
        BIT(E_MOTOR) | BIT(E_RADAR),
        &initWallLeft,
        &deinitWallLeft,
        &testWallLeft
};



//right


void initWallRight(sState *prev) {
    uint16_t tab[RAD_NB_PTS];
    int i;
    for (i=0;i<RAD_NB_PTS;i++){
        tab[i]=15;
    }
    for (i=7;i<=9;i++){
        tab[i]=50;
    }
    radarSetLim(tab);

#ifdef DEBUG
    Serial.println("debut wall right");
#endif
    wallSetVal(RIGHT,WALL_DST,WALL_SPEED);

    if (prev==&sPause) {

#ifdef DEBUG
    Serial.println("\tback from pause");
#endif
        saveTime=millis()-saveTime;
    }
    else{
#ifdef UPPERCUT
    armServoRight.write(ARM_RIGHT_DOWN);
#else
    armServoRight.write(ARM_RIGHT_UP);
#endif
    }
}

void deinitWallRight(sState *next){
    //pause :
    if (next==&sPause) {
        saveTime=millis();
    }
    else {
        saveTime=0;
        move(0,0);
    }
}




sState * testWallRight(){
    static char nblines=0;
    static unsigned long prevMillisWallRight;
    static int armPos1,armPos2;
#ifdef UPPERCUT
    armPos1=ARM_RIGHT_DOWN;
    armPos2=ARM_RIGHT_UP;
#else
    armPos1=ARM_RIGHT_UP;
    armPos2=ARM_RIGHT_DOWN;
#endif
    static int armPos=armPos1;

    if (armPos==armPos2){
        if ((millis()-prevMillisWallRight-saveTime)>ARM_RAISE_TIME) {
            armPos=armPos1;
            armServoRight.write(armPos);
        }
    }
    else if ( getIntensity(4) > LINE_THRESHOLD && getIntensity(3) > LINE_THRESHOLD ) {
        nblines++;
        prevMillisWallRight=millis();
        saveTime=0;
        armPos=armPos2;
        armServoRight.write(armPos);
    }

    if ((millis()-_matchStart) > TIME_MATCH_STOP ) return &sFunny;
    if (nblines>=4) {
        move(0,0);
        if ((millis()-prevMillisWallRight-saveTime)>1000){
            armServoRight.write(ARM_RIGHT_UP);
            return &sTrajBlueFinal;
        }
    }
    if (radarIntrusion()) return &sPause;
    return 0;
}

sState sWallRight={
        BIT(E_MOTOR) | BIT(E_RADAR),
        &initWallRight,
        &deinitWallRight,
        &testWallRight
};








