#ifndef TOOLS_H
#define TOOLS_H

#define BK_GREEN_LED_BOARD 0
#define PIN_GREEN_LED_BOARD 24
#define BK_ORANGE_LED_BOARD 1
#define PIN_ORANGE_LED_BOARD 31

#define M_PI 3.14159265358979323846
#define ROUND(x) (x>0? (long)(x+0.5) : (long)(x-0.5))
#define INCH2METERS 0.0254

float incPerT2mPerS(float incPerT);
float mPerS2IncPerT(float mPerS);
void blindLED(int bk, int pin, int delay_on, int delay_off, int nbr_it /*0 = Infinite time*/);
void switches_init(void);

#endif
