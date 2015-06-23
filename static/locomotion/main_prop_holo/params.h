#ifndef _PARAMS_H
#define _PARAMS_H

#define __USE_GNU
#include <math.h>

/**
 * SHIFTS DEFINITION
 */

// data in the matrix (or its inverse) will be in the range of -3;3, during an inversion or a matrix multiplication up to 3 times an element can be stored
//   ln(3*3)/ln(2) = 3.17 => 4 bits for the integer part + 1 sign bit => 32-(4+1) = 27 bits for the decimal part
//   => 2^27 = 134217728 decimal part possibilities, ln(134217728)/ln(10)=8.13 => more than 8 significant figures for the decimal part
#define MAT_SHIFT (27)
#define dMSHIFT ((double)(1 << MAT_SHIFT))

// data in the vectors will be in the range of -81000;81000
//   ln(81000)/ln(2) = 16.3 => 17 bits for the integer part + 1 sign bit => 32-(18) = 14 bits for the decimal part
//   => 2^14 = 16384 decimal part possibilities, ln(16384)/ln(10)=4.21 => more than 4 significant figures for the decimal part
#define VEC_SHIFT (14)
#define dVSHIFT ((double)(1 << VEC_SHIFT))

#define RAD_SHIFT (13)
#define dRSHIFT ((double)(1 << RAD_SHIFT))

// MAT over RAD shift
#if MAT_SHIFT > RAD_SHIFT
#define dMoRSHIFT ((double)(1 << (MAT_SHIFT - RAD_SHIFT)))
#else
#define dMoRSHIFT ((double)(1 >> (RAD_SHIFT - MAT_SHIFT)))
#endif

// VEC times RAD shift, total shift for angles in radians
#define dASHIFT ((double)(1 << (VEC_SHIFT + RAD_SHIFT)))

// RAD shift, diff shift for angles in radians
#define dRadSHIFT ((double)(1 << RAD_SHIFT))

// variance/uncertainty shifts
#define VAR_POS_SHIFT (1)
#define dVarPosSHIFT ((double)(1 << VAR_POS_SHIFT))

#define VAR_PODS_DATASHIFT (12)
#define dMoVarPodsSHIFT ((double)(1 << (MAT_SHIFT - VAR_PODS_DATASHIFT)))

// M_rob2pods produces (V1;V2;V3) in [IpP<<SHIFT x IpP<<SHIFT x IpP<<SHIFT] from (Vx;Vy;Oz) in [IpP<<SHIFT x IpP<<SHIFT x RpP<<(RAD_SHIFT+SHIFT)]

/**
 * DATA ADAPTATION
 */

#define SHIFT (VEC_SHIFT)
#define dSHIFT ((double)(1<<SHIFT))
#define iROUND(d) ((int)( (d)+0.5 )) // the +0.5 is here to get a round instead of a floor when casting to int
#define isROUND(d) iROUND((d)*dSHIFT)
#define lROUND(d) ((long long)( (d)+0.5 )) // the +0.5 is here to get a round instead of a floor when casting to int
#define lsROUND(d) lROUND((d)*dSHIFT)

#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

#define PI (M_PI)
#define ssPI (PI*dASHIFT) // (rad<<(RAD_SHIFT+SHIFT))
#define issPI iROUND(ssPI)

#define WDIAM (3.25*2.54)  // wheel diameter (cm)

#define SpP (0.02)  // seconds per sampling period
#define MSpP (20.)  // milliseconds per sampling period
#define USpP (20000.)  // microseconds per sampling period
#define IpR (500.*676./49.) // increments per revolution (6897.959183673)
#define DpR (PI*WDIAM) // distance per revolution (25.933847355 cm)

// revolution per second to increments per sampling period
#define RpS2IpP(o) ((o)*IpR*SpP)
#define iRpS2IpP(o) iROUND(RpS2IpP(o))  // (IpP)
#define isRpS2IpP(o) isROUND(RpS2IpP(o)) // (IpP<<SHIFT)

// increments per sampling period to revolution per second
#define IpP2RpS(o) ((o)/IpR/SpP)
#define IpP2RpSs(os) IpP2RpS((o)/dSHIFT)

// centimeters per second to increments per sampling period
#define DpS2IpP(o) RpS2IpP((o)/DpR)
#define iDpS2IpP(o) iROUND(DpS2IpP(o))  // (IpP)
#define isDpS2IpP(o) isROUND(DpS2IpP(o))  // (IpP<<SHIFT)

// increments per sampling period to centimeters per second
#define IpP2DpS(o) (IpP2RpS(o)*DpR)
#define IpP2DpSs(os) (IpP2DpS((os)/dSHIFT))

// centimeters to increments
#define D2I(d) ((d)*IpR/DpR) // (265.982871309 increments per centimeter)
#define iD2I(d) iROUND(D2I(d))  // (I)
#define isD2I(d) isROUND(D2I(d)) // (I<<SHIFT)

// increments to centimeters
#define I2D(I) ((I)*DpR/IpR) // (0.003759641 centimeters per increment)
#define I2Ds(Is) I2D((Is)/dSHIFT)
#define iI2Ds(Is) iROUND(I2Ds(Is))

// rad to increments
#define R2I(R) ((R)/M_TWOPI*IpR)
#define iR2I(R) iROUND(R2I(R)) // (I)
#define isR2I(R) isROUND(R2I(R)) // (I<<SHIFT)

// LPC parameters
#define PWM_RANGE 1024  // If you want to change the resolution/range of the motor command, just don't change this value, you're doing it wrong...

// Parameters for control loops
#define SPEED_NOMI 20 // in cm/s
#define SHIFT_PID_POS (14)
#define SHIFT_PID_SPD (10)

// Simulator parameters (when code running on linux)
#ifdef ARCH_X86_LINUX

// Motor parameter
#define MOT_TIME_CST (50.) // in ms

#endif

#endif

