#ifndef _SHARP_2D120X_H
#define _SHARP_2D120X_H
//#include <stdint.h>
#define NB_SHARP 2
#define DEBUG_SHARP
#ifdef __cplusplus
//extern "C" {
#endif

// returns centimers<<4 when m is from 0 to 1023
int raw2dist120x(int m);
int sharpRead(int pin_sharp);
void setSharpPin(int pinSharp[]);
int sharpIntrusion();
void sharpSetLim(int limits[NB_SHARP]);
#ifdef __cplusplus
//}
#endif

#endif

