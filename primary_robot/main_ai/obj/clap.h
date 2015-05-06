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

        void initObj(Point2D<float> , vector<astar::sObs_t>&, vector<Obj*>&) override {};
        int loopObj(std::vector<astar::sObs_t>& obs, std::vector<uint8_t>& obs_updated, vector<Obj*>&, std::vector<Actuator>&) override;
        eObj_t type() const override {return E_CLAP;};
        //float gain(){return 1;};

    private :
        unsigned int _num;

};

#endif /* OBJ_CLAP_H_ */
