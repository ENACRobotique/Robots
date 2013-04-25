#include <targets/LPC2000.h>

#ifndef luMIN
#define luMIN(a, b) ((a)>(b)?(b):(a))
#endif

#include "pid.h"
#include "pwm.h"
#include "motor.h"
#include "params.h"

#include "controller.h"

pid_t vitMotGauche, vitMotDroit;
motor_t motGauche, motDroit;

void motor_controller_init() {
// asserv
  pid_init(&vitMotGauche, 2<<SHIFT_PID, (2*2/1)<<(SHIFT_PID-3), /*(2*2/1)<<(SHIFT_PID-3)*/0, 900<<SHIFT_PID, SHIFT_PID);
  pid_init(&vitMotDroit, 2<<SHIFT_PID, (2*2/1)<<(SHIFT_PID-3), /*(2*2/1)<<(SHIFT_PID-3)*/0, 900<<SHIFT_PID, SHIFT_PID);

// motors
  motor_init(&motGauche, 4 /* P0.8 */, 1, 22);
  motor_init(&motDroit,  6 /* P0.9 */, 1, 23);
}

#define AMAX 35
#define VMAX iDpS2IpP(60)  // 60cm/s max

void motor_controller_update(int sPL, int pVL, int sPR, int pVR) {
  int value;

// left motor
  // limit acceleration
  if(sPL-pVL>AMAX)
    sPL = pVL+AMAX;
  else if(sPL-pVL<-AMAX)
    sPL = pVL-AMAX;

  // update pid
  value = pid_update(&vitMotGauche, sPL, pVL);

  // limit speed
  if(value>VMAX)
    value = VMAX;
  else if(value<-VMAX)
    value = -VMAX;
  
  // send command
  motor_update(&motGauche, value);

// right motor
  // limit acceleration
  if(sPR-pVR>AMAX)
    sPR = pVR+AMAX;
  else if(sPR-pVR<-AMAX)
    sPR = pVR-AMAX;

  // update pid
  value = pid_update(&vitMotDroit, sPR, pVR);

  // limit speed
  if(value>VMAX)
    value = VMAX;
  else if(value<-VMAX)
    value = -VMAX;
  
  // send command
  motor_update(&motDroit, value);
}
