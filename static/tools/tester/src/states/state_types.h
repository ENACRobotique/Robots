/*
 * types.h
 *
 *  Created on: 23 avr. 2013
 *      Author: quentin
 */

#ifndef TYPES_H_
#define TYPES_H_


typedef enum {
    E_BLINK,
    E_MENU_PRINCIPAL
}eBlocks;

typedef enum {
    SM_NO_MODE,
    SM_TPS_REEL,
    SM_VALID,
    SM_MICROS
}servosModes;

//pointeur de fonction
typedef int(*pfi)();
typedef void(*pfvps)(struct sState*);

typedef struct sState*(*pfps)();


//état élémentaire :
struct sState{
    unsigned int flag;
    pfvps init;
    pfvps deinit;
    pfps testFunction;
};

typedef struct sState sState;


#endif /* TYPES_H_ */
