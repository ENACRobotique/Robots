/*
 * state_traj.cpp
 *
 *  Created on: 8 mai 2013
 *      Author: quentin
 */


#include "Arduino.h"
#include "state_types.h"
#include "lib_move.h"
#include "lib_radar.h"
#include "../tools.h"
#include "../params.h"
#include "state_traj.h"
#include "state_pause.h"
#include "state_wall.h"
#include "state_funny.h"
#include "state_wait.h"

unsigned long st_saveTime=0,st_prevSaveTime=0;


void initTrajBlue(sState *prev) {
    uint16_t tab[RAD_NB_PTS];
    int i;
    for (i=0;i<RAD_NB_PTS;i++){
        tab[i]=15;
    }
    for (i=1;i<6;i++){
            tab[i]=RADAR_SAFETY_DST;
        }
    radarSetLim(tab);
#ifdef DEBUG
    Serial.println("debut traj bleue");
#endif
    if (prev==&sPause) {
#ifdef DEBUG
    Serial.println("\tback from pause");
#endif
        st_saveTime=millis()-st_saveTime+st_prevSaveTime;
    }
    else {
#ifdef UPPERCUT
    armServoRight.write(ARM_RIGHT_DOWN);
#else
    armServoRight.write(ARM_RIGHT_UP);
#endif
    }

}

void deinitTrajBlue(sState *next){
    //pause :
    if (next==&sPause) {
        st_prevSaveTime=st_saveTime;
        st_saveTime=millis();
    }
    else {
        st_saveTime=0;
        st_prevSaveTime=0;
        move(0,0);
    }

}

trajElem start_blue[]={
        {0,0,100}, //to wait until the servo is in the good position
        {100,0,3000}, //avant 29cm
        {0,45,500},
        {100,45,750},
        {0,0,500},
        {100,0,750},
        {0,90,500},
        {-100,90,1000},
        {0,0,100},
        {0,0,0}
};

sState *testTrajBlue(){
    static int i=0;
    static unsigned long prev_millis=0;
    if (periodicProgTraj(start_blue,&st_saveTime,&i,&prev_millis)) return &sWallRight;
    if ((millis()-_matchStart) > TIME_MATCH_STOP ) return &sFunny;
    if (radarIntrusion()) return &sPause;
    return 0;
}

sState sTrajBlue={
        BIT(E_MOTOR) | BIT(E_RADAR),
        &initTrajBlue,
        &deinitTrajBlue,
        &testTrajBlue
};




void initTrajRed(sState *prev) {
    uint16_t tab[RAD_NB_PTS];
    int i;
    for (i=0;i<RAD_NB_PTS;i++){
        tab[i]=15;
    }
    for (i=6;i<RAD_NB_PTS;i++){
            tab[i]=RADAR_SAFETY_DST;
        }
    radarSetLim(tab);
#ifdef DEBUG
    Serial.println("debut traj rouge");
#endif
    if (prev==&sPause) {
#ifdef DEBUG
    Serial.println("\tback from pause");
#endif
        st_saveTime=millis()-st_saveTime+st_prevSaveTime;
    }
    else {
#ifdef UPPERCUT
    armServoLeft.write(ARM_LEFT_DOWN);
#else
    armServoLeft.write(ARM_LEFT_UP);
#endif
    }

}

void deinitTrajRed(sState *next){
    //pause :
    if (next==&sPause) {
        st_prevSaveTime=st_saveTime;
        st_saveTime=millis();
    }
    else {
        st_saveTime=0;
        st_prevSaveTime=0;
        move(0,0);
    }
}

trajElem start_red[]={
        {0,0,100}, //to wait until the servo is in the good position
        {100,0,2800},
        {0,-45,250},
        {50,-45,2000},
        {0,0,250},
        {100,0,500},
        {0,90,250},
        {100,90,800},
        {0,0,100},
        {0,0,0}
};

sState *testTrajRed(){
    static int i=0;
    static unsigned long prev_millis=0;
    if (periodicProgTraj(start_red,&st_saveTime,&i,&prev_millis)) return &sWallLeft;
    if ((millis()-_matchStart) > TIME_MATCH_STOP ) return &sFunny;
    if (radarIntrusion()) return &sPause;
    return 0;
}

sState sTrajRed={
        BIT(E_MOTOR) | BIT(E_RADAR),
        &initTrajRed,
        &deinitTrajRed,
        &testTrajRed
};



void initTrajRedFinal(sState *prev) {
    uint16_t tab[RAD_NB_PTS];
    int i;
    for (i=0;i<=4;i++){
        tab[i]=RADAR_SAFETY_DST;
    }
    for (i=10;i<=11;i++){
            tab[i]=RADAR_SAFETY_DST;
        }
    for (i=4;i<10;i++){
            tab[i]=15;
        }
    radarSetLim(tab);
#ifdef DEBUG
    Serial.println("debut traj finale rouge");
#endif
    if (prev==&sPause) {
#ifdef DEBUG
    Serial.println("\tback from pause");
#endif
        st_saveTime=millis()-st_saveTime+st_prevSaveTime;
    }
    else {
    move(0,0);
    armServoLeft.write(ARM_LEFT_UP);
    delay(500);
    armServoRight.write(ARM_RIGHT_GLASS);
    }

}

void deinitTrajRedFinal(sState *next){
    //pause :
    if (next==&sPause) {
        st_prevSaveTime=st_saveTime;
        st_saveTime=millis();
    }
    else {
        st_saveTime=0;
        st_prevSaveTime=0;
        move(0,0);
    }
}

trajElem final_red[]={
        {0,0,100}, //to wait until the servo is in the good position
        {-100,45,650},
        {-100,0,6000},
        {-100,-2,10000},
        {-100,-45,850},
        {-100,-5,10000},
        {0,0,100},
        {0,0,0}
};

sState *testTrajRedFinal(){
    static int i=0;
    static unsigned long prev_millis=0;
    if (periodicProgTraj(final_red,&st_saveTime,&i,&prev_millis)) return &sWait;
    if ((millis()-_matchStart) > TIME_MATCH_STOP ) return &sFunny;
    if (radarIntrusion()) return &sPause;
    return 0;
}

sState sTrajRedFinal={
        BIT(E_MOTOR)| BIT(E_RADAR),
        &initTrajRedFinal,
        &deinitTrajRedFinal,
        &testTrajRedFinal
};




void initTrajBlueFinal(sState *prev) {
    uint16_t tab[RAD_NB_PTS];
    int i;
    for (i=0;i<=4;i++){
        tab[i]=RADAR_SAFETY_DST;
    }
    for (i=10;i<=11;i++){
            tab[i]=RADAR_SAFETY_DST;
        }
    for (i=4;i<10;i++){
            tab[i]=15;
        }
    radarSetLim(tab);
#ifdef DEBUG
    Serial.println("debut traj finale rouge");
#endif
    if (prev==&sPause) {
#ifdef DEBUG
    Serial.println("\tback from pause");
#endif
        st_saveTime=millis()-st_saveTime+st_prevSaveTime;
    }
    else {
    move(0,0);
    armServoLeft.write(ARM_LEFT_UP);
    delay(500);
    armServoRight.write(ARM_RIGHT_GLASS);
    }

}

void deinitTrajBlueFinal(sState *next){
    //pause :
    if (next==&sPause) {
        st_prevSaveTime=st_saveTime;
        st_saveTime=millis();
    }
    else {
        st_saveTime=0;
        st_prevSaveTime=0;
        move(0,0);
    }
}

trajElem final_blue[]={
//        {0,0,100}, //to wait until the servo is in the good position
//        {-100,-45,1000},
//        {-100,0,15000},
//        {-100,45,500},
//        {-100,0,14000},
//        {0,0,0}
        {0,0,100}, //to wait until the servo is in the good position
        {-100,-45,1600},
        {-100,0,10000},
        {-100,45,1000},
        {-100,-2,16000},
        {0,0,0}

};

sState *testTrajBlueFinal(){
    static int i=0;
    static unsigned long prev_millis=0;
    if (periodicProgTraj(final_blue,&st_saveTime,&i,&prev_millis)) return &sWait;
    if ((millis()-_matchStart) > TIME_MATCH_STOP ) return &sFunny;
    if (radarIntrusion()) return &sPause;
    return 0;
}





sState sTrajBlueFinal={
        BIT(E_MOTOR) | BIT(E_RADAR),
        &initTrajBlueFinal,
        &deinitTrajBlueFinal,
        &testTrajBlueFinal
};




int periodicProgTraj(trajElem tab[],unsigned long *pausetime, int *i, unsigned long *prev_millis){
    if (!(*prev_millis)) *prev_millis=millis();
    move(tab[*i].speed,tab[*i].angle);
    if ( (millis()-*prev_millis-*pausetime)>tab[*i].duration ) {
        (*i)++;
        *prev_millis=millis();
        *pausetime=0;
    }
    if ( tab[*i].angle==0 && tab[*i].duration==0 && tab[*i].speed==0) {
        *i=0;
        *prev_millis=0;
        return 1;
    }
    return 0;
}


