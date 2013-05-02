#ifndef _PARAMS_H
#define _PARAMS_H

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

void init_globals();

#endif

