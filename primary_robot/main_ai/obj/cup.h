/*
 * cup.h
 *
 *  Created on: 21 avr. 2015
 *      Author: Sebastien Malissard
 */

#ifndef OBJ_CUP_H_
#define OBJ_CUP_H_


#include <types.h>
#include "obj.h"
#include "tools.h"

#define START_CUP   20 //number of the first stand element in obs[]

using namespace std;

class Cup : public Obj{
    public:
        Cup(const unsigned int num, vector<astar::sObs_t>& obs);
        virtual ~Cup();

        void initObj(){};
        int loopObj();
        eObj_t type() const {return E_CUP;};

    private :
        unsigned int _num;
};



#endif /* OBJ_CUP_H_ */