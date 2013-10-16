#ifndef _CONTROLLER_H
#define _CONTROLLER_H

#define AMAX (35)
#define VMAX iDpS2IpP(60.)  // 60cm/s max

void motor_controller_init();
void motor_controller_update(int sPL, int pVL, int sPR, int pVR);

#endif