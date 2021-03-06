#include <math.h>

#include "params.h"

sGlobParams glob_params;

void init_globals(sideColor c, sPt_t *x0) {
	glob_params.omega = 20.*2.*M_PI; // (rad/s) TODO update according to latest period
	glob_params.r = 25e-3; // (m)
	glob_params.epsilon = 0.5*M_PI/180.; // (rad)

	glob_params.u_delta_t_distance = 8e-6; // (s)
	glob_params.u_delta_t_angle = 16e-6; // (s)
	glob_params.u_omega = SQR(glob_params.omega)/(2.*M_PI)*4e-6; // (rad/s)

	// cas cote rouge
	if(c == RED){
		glob_params.B1.x = -0.04;
		glob_params.B1.y = -0.04;
		glob_params.B2.x = -0.04;
		glob_params.B2.y =  2.04;
		glob_params.B3.x =  3.04;
		glob_params.B3.y =  1.;
		x0->x = 1.8;
		x0->y = 0.2;
	}
	// cas cote jaune
	else{
		glob_params.B1.x =  3.04;
		glob_params.B1.y = -0.04;
		glob_params.B2.x =  3.04;
		glob_params.B2.y =  2.04;
		glob_params.B3.x = -0.04;
		glob_params.B3.y =  1.;
		x0->x = 1.8;
		x0->y = 2.8;
	}
	glob_params.D12 = sqrt(SQR(glob_params.B2.x - glob_params.B1.x) + SQR(glob_params.B2.y - glob_params.B1.y));
	glob_params.D23 = sqrt(SQR(glob_params.B3.x - glob_params.B2.x) + SQR(glob_params.B3.y - glob_params.B2.y));
	glob_params.D31 = sqrt(SQR(glob_params.B1.x - glob_params.B3.x) + SQR(glob_params.B1.y - glob_params.B3.y));
}

