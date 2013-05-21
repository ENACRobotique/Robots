#include <targets/LPC2000.h>
#include <limits.h>

#include "pid.h"

int pid_init(pid_t *p, unsigned int KP, unsigned int KI, unsigned int KD, unsigned int I_max, unsigned char shift) {
  p->shift = shift;

  p->KP = KP; // Proportional factor (KP)
  p->KI = KI; // Integral factor (KP*T/Ti)
  p->KD = KD; // Derivative factor (KP*Td/T)
  p->I_max = I_max;

  pid_reset(p);
}

void pid_reset(pid_t *p) {
  p->maxErr = INT_MAX / (p->KP + 1);

  p->sum = 0;
  if(p->KI)
    p->maxSum = p->I_max / p->KI;
  else {
    p->maxSum = 0;
    p->I_max = 0;
  }
  p->lastProcessValue = 0;
}

int pid_update(pid_t *p, int setPoint, int processValue) {
  long long err, temp, ret;

// input error
  err = setPoint - processValue;

// Proportional term
  if(err > p->maxErr)
    ret = INT_MAX;
  else if(err < -p->maxErr)
    ret = INT_MIN;
  else
    ret = err * (int)p->KP;

// Integral term
  temp = p->sum + err;
  if(temp > p->maxSum) {
    p->sum = p->maxSum;
    ret += p->I_max;
  }
  else if(temp < -p->maxSum) {
    p->sum = -p->maxSum;
    ret += -p->I_max;
  }
  else {
    p->sum = temp;
    ret += p->sum * (int)p->KI;
  }

// Derivative term
  ret += (p->lastProcessValue - processValue) * (int)p->KD;
  p->lastProcessValue = processValue;

// clamp output to avoid overflow
  ret >>= p->shift;
  if(ret > INT_MAX)
    ret = INT_MAX;
  else if(ret < INT_MIN)
    ret = INT_MIN;

  return (int)ret;
}
