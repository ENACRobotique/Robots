/*
 * clap.hpp
 *
 *  Created on: 30 janv. 2015
 *      Author: seb
 */

#ifndef OBJ_CLAP_H_
#define OBJ_CLAP_H_


#include <types.h>
#include "obj.h"

using namespace std;

class Clap : public Obj{
    public:
        Clap(const unsigned int num);
        virtual ~Clap();

        void initObj(paramObj) ;
        int loopObj(paramObj) ;
        eObj_t type() const  {return E_CLAP;};
        //float gain(){return 1;};

    private :
        unsigned int _num;
        Point2D<float> _dest;

};

#endif /* OBJ_CLAP_H_ */
