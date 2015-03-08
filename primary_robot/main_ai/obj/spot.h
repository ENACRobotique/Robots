/*
 * clap.hpp
 *
 *  Created on: 30 janv. 2015
 *      Author: seb
 */

#ifndef OBJ_SPOT_H_
#define OBJ_SPOT_H_


#include <types.h>
#include "obj.h"

#define START_STAND 4 //number of the first stand element in obs[]

using namespace std;

class Spot : public Obj{
    public:
        Spot(const unsigned int num);
        virtual ~Spot();

        void initObj(){};
        int loopObj();
        eObj_t type() const {return E_SPOT;};

    private :
        unsigned int _num;

};

#endif /* OBJ_CLAP_H_ */
