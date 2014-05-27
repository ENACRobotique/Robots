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

float curDate = 0;
float nbFire = 0;   //with the curDate
int nbFireRecv = 0;



void newMsgZone(sPt_t pt[], int nb){
    int i, ret;

    for( i = 0 ; i < NB_OBJ ; i++){
         if(listObj[i].etype == E_FEU){
             if(listObj[i].state == WAIT_MES){ //We don't have new position between two scan zone
                 listObj[i].state = FREE;
                 listObj[i].nbObs = 1;
                 listObj[i].active = 0;
                 }
             testPtInZone(&pt[nb], nb, &obs[listObj[i].numObs[0]].c, &ret);
             if(ret){
                 listObj[i].state = WAIT_MES;
                 }
             }
         }
    }

void checkMsgZone(){
    sGenericStatus *msg;
    static int nbpt = 0;
    static float date_last_msg;
    static sPt_t tab[NB_MAX_PT_ZONE];

    msg = getLastPGStatus(ELT_ZONE);

    while(msg != NULL){
        if(msg->date > curDate){
            if(msg->date == date_last_msg){
                if(msg->zone_status.pos.frame == FRAME_PRIMARY){
                    //TODO change the repository
                    }
                tab[nbpt].x = msg->zone_status.pos.x;
                tab[nbpt].y = msg->zone_status.pos.y;
                nbpt++;
                }
            else if(msg->date > date_last_msg){ //ignore old message
                nbpt = 0;
                date_last_msg = msg->date;
                if(msg->zone_status.pos.frame == FRAME_PRIMARY){
                    //TODO change the repository
                    }
                tab[nbpt].x = msg->zone_status.pos.x;
                tab[nbpt].y = msg->zone_status.pos.y;
                nbpt++;
                }
            if(nbpt == msg->zone_status.nbpt){
                curDate = msg->date;
                nbFire = msg->zone_status.nbfire;
                newMsgZone(tab, msg->zone_status.nbpt);
                }
            }
        msg = getLastPGStatus(ELT_ZONE);
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
    sGenericStatus *mes = NULL;
    sPt_t pt;

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
            nbFireRecv++;
            }
        } while(mes != NULL);

    return 1;
    }

void getStatusFire(void){
    int i;
    sMsg msgOut;

    if(nbFire == nbFireRecv){
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

    msgOut.header.destAddr = ADDRD2_MAIN_VIDEO;
    msgOut.header.type = E_GENERIC_STATUS;
    msgOut.header.size = 5 + sizeof(msgOut.payload.genericStatus.fire_status);

    msgOut.payload.genericStatus.id = ELT_FIRE;

    bn_send(&msgOut);
    }



