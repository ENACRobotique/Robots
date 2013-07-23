#ifndef _SHARP_2D120X_H
#define _SHARP_2D120X_H

#ifdef __cplusplus
extern "C" {
#endif

// returns centimers<<4 when m is from 0 to 1023
int raw2dist120x(int m);

#ifdef __cplusplus
}
#endif

#endif

