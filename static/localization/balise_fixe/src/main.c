#define BUFFER_LENGTH 8

#include "inc/hw_ints.h"
#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/rom.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#include "time.h"
#include "neldermead.h"
#include "stdint.h"
#include "perception.h"
#include "inc/lm4f120h5qr.h"
#include <driverlib/fpu.h>
#include "tools.h"


#ifndef BIT
#define BIT(a) (1<<a)
#endif

//XXX #define RANGE
typedef enum{INIT,COLORDETEC,PLAY} EBaliseState;

/*
 * !!! ATTENTION !!!
 * 	tab[1] pour acceder à un el
 * 	-2 mod 3 = -2
 *copier neldermead and co
 */
/*
 *  typedef struct {
 *		unsigned long deltaT;       // µs, delay between two laser small peaks
 *	 	unsigned long date;         // local µs, when was the laser recorded last
 *		unsigned long thickness;    // µs, thickness of the small laser peak /!\ thickness==0 <=> no laser detected
 *		unsigned long period;       // µs, MEASURED period (0 if not applicable).
 *		int precision;              // xxx TDB
 *		long int sureness;          // TBD
 *	}plStruct;
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

/* XXX buffer = tableau de plStruc ou plStruc*
 *
 * 	?type buffer[BUFFER_LENGTH];
 *
 */



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
// computes sPerception from 3 plStruc TODO

/*sPerception calcPerception( buffertype * header){
 *
 * }
 */

// return apporximated position from the two last positions
void approxPos(sPt_t *Z1, sPt_t *Z2, sPt_t *res){
	res->x = 2*Z2->x - Z1->x;
	res->y = 2*Z2->y - Z1->y;
}
// main function.
int main(void) {
	FPUEnable();
	EBaliseState state = INIT;
//	sPt_t LastPos[2];
//	sPt_t x0;

	// Initialisation

	// Loop forever.
	while(1){
		switch(state){
		case INIT:
			// wait for 3 available info
			/* if (buffer[(head - 3)&BUFFER_LENGTH]->num_balise != 0) {
			 * 		state = COLORDETEC;
			 * }
			 */
			break;
		case COLORDETEC:
			// Determine team's color
	/*
	 * 		// current head back_up
	 * 		int hd_tmp = head;
	 *
	 * 		//test that the 3 last informations are from different beacons
	 * 		if (BIT(buffer.(hd_tmp)->num_balise)
	 * 			+ BIT(buffer.((hd_tmp - 1)&BUFFER_LENGTH)->num_balise)
	 * 			+ BIT(buffer.((hd_tmp - 2)&BUFFER_LENGTH)->num_balise)
	 * 			= 7){
	 *
	 * 			//test if all beacon were shot in the same lasor rotation period
	 * 			if( (buffer.(hd_tmp)->t - buffer.((hd_tmp - 2)&BUFFER_LENGTH)->t) < T_laser){
	 * 				int test_result = 1;
	 * 				/*
	 * 				 *if team red possible beacon number orders
	 * 				 *  1 2 3 ; 2 3 1; 3 1 2
	 * 				 *
	 * 				 * if team yellow
	 * 				 *	3 2 1 ; 2 1 3; 1 3 2
	 * 				 *
	 * 				 * test verified by red orders and not by yellow ones
	 * 				 *  n.(i) [3] < n.(i+1)
	 * 				 * /
	 * 				for(i = hd_tmp - 2; i <hd_tmp; i++){
	 * 					team_color = 1;
	 * 					test_result = test_result & ( (buffer.(i)->num_balise % 3 )< buffer.(i+1)->num_balise
	 * 				}
	 * 				if (test_result){
	 * 					TODO team  RED
	 * 				}
	 * 				else{
	 * 					TODO team YELLOW
	 * 				}
	 * 				state = PLAY;
	 * 				for(i=0 ; i<2;i++){
	 * 					ask_position_to_base_roulante -> LastPos TODO
	 * 				}
	 * 		}
	 */
			break;
		case PLAY:
		/* // current head back_up
		 * int hd_tmp = head;
		 *
		 * //test that the 3 last informations are from different beacons
		 * if (BIT(buffer.(hd_tmp)->num_balise)
		 * 		+ BIT(buffer.((hd_tmp - 1)&BUFFER_LENGTH)->num_balise)
		 * 		+ BIT(buffer.((hd_tmp - 2)&BUFFER_LENGTH)->num_balise)
		 * 		= 7){
		 *
		 * 		//test if all beacon were shot in the same lasor rotation period
		 * 		if( (buffer.(hd_tmp)->t - buffer.((hd_tmp - 2)&BUFFER_LENGTH)->t) < T_laser){
		 * 			sPerception perception = calcPerception(hd_tmp);
		 * 			x0 = approxPos(LastPos[0],LastPos[1]);
		 * 			if (neldermead(&x0, RANGE, perception){
		 * 				LastPos[0] = LastPos[1];
		 * 				LastPos[1] = *x0;
		 * 				TODO renvoi position
		 * 			}else{
		 * 				LastPos[0] = LastPos[1];
		 * 				LastPos[1] = ??? XXX TBD XXX
		 * 			}
		 * 		}
		 * 	}

		 */
			break;

		}
	}



}

