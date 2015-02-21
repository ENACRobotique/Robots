#ifndef _PARAMS_H
#define _PARAMS_H

#include <math.h>

#define MAT_SHIFT (16)
#define dMSHIFT ((double)(1 << MAT_SHIFT))

#define VEC_SHIFT (16)
#define dVSHIFT ((double)(1 << VEC_SHIFT))

#define SHIFT (8)
#define SHIFT_PID (8)
#define dSHIFT ((double)(1<<SHIFT))
#define iROUND(d) ((int)( (d)+0.5 )) // the +0.5 is here to get a round instead of a floor when casting to int
#define isROUND(d) iROUND((d)*dSHIFT)
#define lROUND(d) ((long long)( (d)+0.5 )) // the +0.5 is here to get a round instead of a floor when casting to int
#define lsROUND(d) lROUND((d)*dSHIFT)

#define PI (M_PI)
#define sPI (PI*dSHIFT)
#define isPI (iROUND(sPI))  // (rad<<SHIFT)

#define WDIAM (3.25*2.54)  // wheel diameter (cm)

#define SpP (0.02)  // seconds per sampling period
#define IpR (500.*676./49.) // increments per revolution
#define DpR (PI*WDIAM) // distance per revolution (cm)

// revolution per second to increment per sampling period
#define RpS2IpP(o) ((o)*IpR*SpP)
#define iRpS2IpP(o) iROUND(RpS2IpP(o))  // (IpP)
#define isRpS2IpP(o) isROUND(RpS2IpP(o)) // (IpP<<SHIFT)

// centimeters per second to increment per sampling period
#define DpS2IpP(o) RpS2IpP((o)/DpR)
#define iDpS2IpP(o) iROUND(DpS2IpP(o))  // (IpP)
#define isDpS2IpP(o) isROUND(DpS2IpP(o))  // (IpP<<SHIFT)

// centimeters to increments
#define D2I(d) ((d)*IpR/DpR)
#define iD2I(d) iROUND(D2I(d))  // (I)
#define isD2I(d) isROUND(D2I(d)) // (I<<SHIFT)

// increments to centimeters
#define I2D(I) ((I)*DpR/IpR)
#define I2Ds(Is) I2D((Is)/dSHIFT)
#define iI2Ds(Is) iROUND(I2Ds(Is))

// rad to increments
#define R2I(R) ((R)/M_TWOPI*IpR)
#define iR2I(R) iROUND(R2I(R)) // (I)
#define isR2I(R) isROUND(R2I(R)) // (I<<SHIFT)


#define PWM_RANGE 1024  // If you want to change the resolution of the motor command, just don't change this value, you're doing it wrong...

// Parameters for control loops
#define PER_ASSER 20000 // in µs
#define PER_ASSER_CRITIC 30000 // in µs
#define SPEED_NOMI 20 // in cm/s
//#define MAX_SPEED 2 // in m/s FIXME
//#define MAX_ACC   1 // in m/s²
//#define MAX_ANG_SPD 180 // in deg/s
//#define MAX_ANG_ACC 90 // in deg/s²

#endif

