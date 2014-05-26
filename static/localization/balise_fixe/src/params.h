/*
 * params.h
 *
 *  Created on: 4 mai 2013
 *      Author: quentin
 */

#ifndef PARAMS_H_
#define PARAMS_H_

#include "node_cfg.h"

typedef enum{
    S_BEGIN,
    S_CHANNEL,        // Initial channel selection
    S_SYNC_ELECTION,  // Sync laser interruption election
    S_SYNC_MEASURES,  // Clock drift measurement
    S_SYNCED,         // Waiting until everybody is synced (message from turret tells us)
    S_GAME            // Game mode
} mainState;

typedef enum{
	RED,
	YELLOW
}sideColor;

#define SENDING_PERIOD 1100 //in ms

//#define DEBUG
//#define DEBUG_SYNC
//#define DEBUG_SYNC_VALUES
#define VERBOSE_SYNC

#if MYADDRX==ADDRX_FIX
    #define HARDUPDATEPERIOD  0         // 1/abs(delta) or O if disabled. delta is the first order drift between the turret and the considered beacon)
    #define HARDUPDATESIGN    0         // sgn(delta)
//FIXME !!!
#warning "compute delta"
#endif

#include "tools.h"

typedef struct {
    // position balises (à calibrer à chaque début de match)
    sPt_t B1, B2, B3;

    float D12;  // distance B1-B2
    float D23;  // distance B2-B3
    float D31;  // distance B3-B1

    float omega;    // vitesse de rotation du rotor (rad/s)
    float r; // rayon du cercle sur lequel sont les deux lasers (m)
    float epsilon;  // erreur de parallélisme des lasers (rad)

    // paramètres d'incertitude
    float u_delta_t_distance;  // (s)
    float u_delta_t_angle;  // (s)
    float u_omega;  // (rad/s)
} sGlobParams;

extern sGlobParams glob_params;

void init_globals(sideColor c, sPt_t *x0);

#endif /* PARAMS_H_ */
