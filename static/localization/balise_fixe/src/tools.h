#ifndef _TOOLS_H
#define _TOOLS_H

#define SQR(v) ((v)*(v))
#define MIN(a, b) ((a)>(b)?(b):(a))
#define MAX(a, b) ((a)>(b)?(a):(b))

typedef struct {
    float x;
    float y;
} sPt_t;
typedef sPt_t sVec_t;

float uni_rand(float min, float max);

#endif

