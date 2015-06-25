/*
 * lib_encoder.h
 *
 *  Created on: 23 juin 2015
 *      Author: Fab
 */

#ifndef LIB_ENCODER_H_
#define LIB_ENCODER_H_

#define CLAMP(m, n, M) min(max((m), (n)), (M))

class Encoder
{
    public:

	Encoder(int p1, int p2);

    int read();

    void write(int nb);

    void init();

    void setLimits(int min, int max);

    void setMultiplicators(int base_multiplicator, int fast_multiplicator);

    void update();

    private:

    int position;

    int pin1;
    int pin2;

    int min;
    int max;

    int base_inc;
    int fast_inc;
    unsigned long prev;
};




#endif /* LIB_ENCODER_H_ */
