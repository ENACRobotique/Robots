/*
 * absolutepos.c
 *
 *  Created on: 25 mai 2014
 *      Author: ThomasDq
 */

#include <absolutepos.h>
#include <lib_synchro_beacon.h>
#include <messages.h>
#include <network_cfg.h>
#include <params.h>

#include "../../../communication/botNet/shared/message_header.h"

typedef enum{INIT,COLORDETEC,PLAY} EBaliseState;
EBaliseState state = INIT;


/*
 * TODO calcperception
 * TODO sendmessage (sendrole)

 */

/*
typedef union{
    uint8_t raw[BN_MAX_PDU-sizeof(sGenericHeader)];		//only used to access data/data modification in low layer
    uint8_t data[BN_MAX_PDU-sizeof(sGenericHeader)];	//arbitrary data, actual size given by the "size" field of the header
    uint8_t debug[BN_MAX_PDU-sizeof(sGenericHeader)];   //debug string, actual size given by the "size" field of the header
    sAckPayload ack;
    sRoleSetupPayload roleSetup;
    sINTP   intp;

 *********************** user payload start ***********************
//the user-defined payloads from above must be added here. The simple ones can be directly added here
//Warning : the user has to make sure that these payloads are not too big (cf BN_MAX_PDU)
    uint8_t channel;
    uint32_t period;
    sTrajElRaw_t traj;
    sPosPayload pos;
    sMobileReportPayload mobileReport;
    sSyncPayload sync;
    sAsservStats asservStats;
    sObsConfig obsCfg;
    sObss obss;
    sGenericStatus genericStatus;
    sPosQuery posQuery;
    sServos servos;
    sIhmStatus ihmStatus;
    sSpeedSetPoint speedSP;
 *********************** user payload stop ***********************

}uPayload;


//final message structure
typedef struct{
    sGenericHeader header;
    uPayload payload;
}sMsg;*/




/* (laser tourne en sens horaire)
 *
 * cas rouge
 * 2--/--------------\--|
 * |R/                \J|
 * |/                  \3    => 1 2 3
 * |                    |
 * 1--------------------|
 *
 * cas jaune
 * |--/--------------\--2
 * |R/                \J|
 * 3/                  \|    => 3 2 1
 * |                    |
 * |--------------------1
 *
 * ^
 * |--/--------------\--|
 * Y R                J |
 * |/                  \|
 * |                    |
 * |--------------------X->
 *
 *
 */
typedef struct{
	sPt_t pt;
	unsigned long date;
}sDatedPt_t;

// return apporximated position from the two last positions
void approxPos(unsigned long current_date, sDatedPt_t *Z1, sDatedPt_t *Z2, sPt_t *res){
	res->x = Z2->pt.x + (current_date - Z2->date) / (Z2->date - Z1->date) * (Z2->pt.x - Z1->pt.x);
	res->y = Z2->pt.y + (current_date - Z2->date) / (Z2->date - Z1->date) * (Z2->pt.y - Z1->pt.y);
}


void absolutepos(sMeasures*buffer,int index, int taille) {

	sDatedPt_t LastPos[2];
	sPt_t x0;
	int i;
	int idx[3];
	idx[0] = (taille + index - 3)%taille;
	idx[1] = (taille + index - 2)%taille;
	idx[2] = (taille + index - 1)%taille;

	// Initialisation

	switch(state){
	case INIT:
		// wait for 3 available info
		if (buffer[idx[0]].beacon != 0) {
			state = COLORDETEC;
		}

		break;
	case COLORDETEC:
		// Determine team's color

		// current head back_up

		//test that the 3 last informations are from different beacons
		if (BIT(buffer[idx[1]].beacon)
				+ BIT(buffer[idx[2]].beacon)
				+ BIT(buffer[idx[3]].beacon)
				== 7){

			//test if all beacon were shot in the same lasor rotation period
			if( (buffer[idx[3]].date - buffer[idx[1]].date)
					< (buffer[idx[3]].period + (buffer[idx[3]].period>>1)) ){
				int test_result = 1;
				/*
				 *if team red possible beacon number orders
				 *  1 2 3 ; 2 3 1; 3 1 2
				 *
				 * if team yellow
				 *	3 2 1 ; 2 1 3; 1 3 2
				 *
				 * test verified by red possible orders and not by yellow ones
				 *  n.(i) [3] < n.(i+1)
				 */
				for( i = 0; i <3; i++){
					test_result = test_result & ( (buffer[idx[i]].beacon % 3) < buffer[idx[(i+1)%3]].beacon);
				}
				// cas RED
				if (test_result){
					init_globals(RED, &x0);

				}
				// cas YELLOW
				else{

					init_globals(YELLOW, &x0);
				}
				state = PLAY;
				for(i=0 ; i<2;i++){
					LastPos[i].pt = x0; // XXX C'est pas bô
				}
			}

			break;
	case PLAY:

		//test that the 3 last informations are from different beacons
		if (BIT(buffer[idx[0]].beacon)
				+ BIT(buffer[idx[1]].beacon)
				+ BIT(buffer[idx[2]].beacon)
				== 7){

			//test if all beacon were shot in the same lasor rotation period ( < 1.5 period)
			if( (buffer[idx[2]].date - buffer[idx[0]].date)
					< (buffer[idx[0]].period + (buffer[idx[0]].period>>1))
			){
				sPerception perception;
				calcPerception(&perception,&(buffer[idx[0]]),&(buffer[idx[1]]),&buffer[idx[2]]);
				approxPos( buffer[idx[2]].date, &LastPos[0],&LastPos[1], &x0);
				if (neldermead(&x0, RANGE, &perception)){
					LastPos[0] = LastPos[1];
					LastPos[1].pt = x0;
					LastPos[1].date = buffer[idx[2]].date;

					//TODO renvoi position
					sMsg msg_out;

						// Header
					msg_out.header.type = E_GENERIC_STATUS;
					msg_out.header.size = sizeof(sGenericStatus);

						// Payload
					msg_out.payload.genericStatus.date = micros2s(buffer[idx[2]].date);
					msg_out.payload.genericStatus.id = ELT_PRIMARY;
					msg_out.payload.genericStatus.pos.x = x0.x;
					msg_out.payload.genericStatus.pos.y = x0.y;
					msg_out.payload.genericStatus.pos.frame = FRAME_PLAYGROUND;
					msg_out.payload.genericStatus.pos_u.a_std = 0;
					msg_out.payload.genericStatus.pos_u.b_std = 0;
					msg_out.payload.genericStatus.pos_u.a_angle = 0;
					msg_out.payload.genericStatus.pos_u.theta = -1;

					rolesend(msg_out);
				}else{
					LastPos[0] = LastPos[1];
				}
			}
		}

		break;

		}
	}
}


