#ifndef _PARAMS_H
#define _PARAMS_H

#define SHIFT (8)
#define SHIFT_PID (8)
#define dSHIFT ((double)(1<<SHIFT))
#define iROUND(d) ((int)( (d)+0.5 )) // the +0.5 is here to get a round instead of a floor when casting to int
#define isROUND(d) iROUND((d)*dSHIFT)
#define lROUND(d) ((long long)( (d)+0.5 )) // the +0.5 is here to get a round instead of a floor when casting to int
#define lsROUND(d) lROUND((d)*dSHIFT)

#define PI (3.141592654)
#define sPI (PI*dSHIFT)
#define isPI (iROUND(sPI))  // (rad<<SHIFT)
#define RPI (D2I(RDIAM)*PI)
#define sRPI (RPI*dSHIFT)
#define iRPI (iROUND(RPI))  // (rad.I)
#define isRPI (iROUND(sRPI))  // (rad.I<<SHIFT)

#define WDIAM (9.95)  // wheel diameter (cm)
#define RDIAM (25.9)  // robot diameter (inter-wheels distance) (cm)

#define SpP (0.02)  // seconds per sampling period
#define IpR (10125.) // increments per revolution
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

#endif
