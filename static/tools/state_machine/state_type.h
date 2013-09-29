//Important notice !
//This file is an example
//Do not use "as is". This code must be pasted in your project and modified to match your needs.


#ifndef STATE_TYPE_H_
#define STATE_TYPE_H_


//blocks that may be called in several states
typedef enum {
	E_GENERICBLOCK
}eBlocks;

//function pointers
typedef int(*pfi)();
typedef void(*pfvps)(struct sState*);

typedef struct sState*(*pfps)();


//state-defining structure :
struct sState{
    unsigned int flag;
    pfvps init;
    pfvps deinit;
    pfps testFunction;
};
typedef struct sState sState;


#endif /* STATE_TYPE_H_ */
