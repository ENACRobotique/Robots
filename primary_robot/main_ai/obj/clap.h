/*
 * clap.hpp
 *
 *  Created on: 30 janv. 2015
 *      Author: seb
 */

#ifndef OBJ_CLAP_H_
#define OBJ_CLAP_H_

#include <main.h>
#include <types.h>
#include "obj.h"

using namespace std;

class Clap : public Obj{
    public:
        Clap();
        virtual ~Clap();

        void initObj(){};
        int loopObj(){return 1;};
        eObj_t type() const {return E_CLAP;};
        float gain(){return 1;};

};

#endif /* OBJ_CLAP_H_ */
