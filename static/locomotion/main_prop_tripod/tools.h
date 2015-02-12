#ifndef TOOLS_H
#define TOOLS_H



#define M_PI 3.14159265358979323846
#define ROUND(x) (x>0? (long)(x+0.5) : (long)(x-0.5))
#define iROUND(d) ((int)( (d)+0.5 )) // the +0.5 is here to get a round instead of a floor when casting to int
#define INCH2METERS 0.0254

#define SIGN(a) ((a)>=0 ? 1 : -1)

float incPerT2mPerS(float incPerT);
float mPerS2IncPerT(float mPerS);
void switches_init(void);

#endif
