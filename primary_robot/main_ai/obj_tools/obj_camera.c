/*
 * obj_camera.c
 *
 *  Created on: 3 mai 2014
 *      Author: seb
 */

#include "obj_camera.h"
#include "messages.h"
#include "obj_types.h"
#include "obj_fct.h"


void receptMsgZone(sPt_t *pt[], int nb){
    int i, ret;

    for( i = 0 ; i < NB_OBJ ; i++){
         if(listObj[i].etype == E_FEU){
             if(listObj[i].state == WAIT_MES){ //We don't have new position between two scan zone
                 listObj[i].state = FREE;
                 listObj[i].nbObs = 1;
                 listObj[i].active = 0;
                 }
             testPtInZone(pt[nb], nb, &obs[listObj[i].numObs[0]].c, &ret);
             if(ret){
                 listObj[i].state = WAIT_MES;
                 }
             }
         }
    }

int updateFire(sPt_t *p, uint8_t or, sNum_t angle){ //or : orientation of the fire. Use this function after receptZone
    int i, j;
    sNum_t d;

    for( i = 0 ; i < NB_OBJ ; i++){
        if((listObj[i].etype == E_FEU) && (listObj[i].state == WAIT_MES)){
            distPt2Pt(p, &obs[listObj[i].numObs[0]].c, &d);
            if( d < PRES_CAM_FIRE){
                if( listObj[i].utype.fire.pos == or){ //same position
                    return 1; //we don't change the fire
                    }
                listObj[i].utype.fire.angle = angle;
                listObj[i].utype.fire.pos = or;
                listObj[i].utype.fire.c = *p;
                obs[listObj[i].numObs[0]].c = *p;
                return 1;
                }
            }
        }

    for( i = 0 ; i < NB_OBJ ; i++){ //No found similar fire -> add to free location
        if(listObj[i].state == FREE){
            listObj[i].nbObs = 1;
            listObj[i].utype.fire.pos = or;
            listObj[i].utype.fire.angle = angle;
            listObj[i].utype.fire.nb_point = 2;
            listObj[i].utype.fire.c = *p;
            listObj[i].nbEP=3;
            listObj[i].state = ACTIVE;
            createEPfire2(i);

            for(j = 0 ; j < N ; j++){
                if(obs[j].state == 0){
                    obs[j].state = 1;
                    obs[j].c = *p;
                    obs[j].moved = 1;
                    obs[j].r = R_ROBOT+7;
                    obs[j].active = 1;
                    listObj[i].numObs[0] = j;
                    }
                }
            return 1;
            }
       }

    return -1; //no free place
    }

int checkMsgFire(void){
    int i;
    static float curDate = 0;
    static int ctFire = 0; //link with the current date
    static int nbFire = 0;  //link with the current date
    sGenericStatus *mes = NULL;
    sGenericZone *zone = NULL;
    sPt_t pt, p[8];

    //check if a new zone message
    if(/*TODO message zone fire receive getLastPGZone(ELT_FIRE)*/1){
        if(zone->date > curDate){ //message more recent receive
            for(i = 0 ; i < zone->nbpt ; i++){
                if(zone->pos[i].frame == FRAME_PRIMARY){
                    //TODO change the repository and angle
                    }
                p[i].x = zone->pos[i].x;
                p[i].y = zone->pos[i].y;
                }
            receptMsgZone(p, zone->nbpt);//FIXME warning
            if( nbFire == ctFire){
                for(i = 0 ; i < NB_OBJ ; i++){
                    if((listObj[i].etype == E_FEU) && (listObj[i].state == WAIT_MES)){
                        listObj[i].state = FREE;
                        obs[listObj[i].numObs[0]].state = 0;
                        }
                    }
                }
            else{ //check if before some messages are lost
                for(i = 0 ; i < NB_OBJ ; i++){
                    if((listObj[i].etype == E_FEU) && (listObj[i].state == WAIT_MES)){
                        listObj[i].state = ACTIVE;
                        obs[listObj[i].numObs[0]].state = 1;
                        }
                    }
                }
            curDate = zone->date;
            ctFire = 0;
            nbFire = zone->fire_zone.nbfire;
            }
        }

    //check if new message fire
    do{
        mes = getLastPGStatus(ELT_FIRE);
        if(mes->date == curDate){
            if(mes->fire_status.pos.frame == FRAME_PRIMARY){
                //TODO change the repository and angle
                }
            pt.x = mes->fire_status.pos.x;
            pt.y = mes->fire_status.pos.y;
            updateFire(&pt, mes->fire_status.state, mes->fire_status.pos.theta);
            nbFire++;
            }
        } while(mes != NULL);

    return 1;
    }

void getStatusFire(void){

    //TODO send a message for ask a photo

    }



