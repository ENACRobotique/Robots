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
#include "tools.h"

#define START_STAND 4 //number of the first stand element in obs[]

using namespace std;

class Spot : public Obj{
    public:
        Spot(const unsigned int num, eColor_t color, vector<astar::sObs_t>& obs);
        virtual ~Spot();

        void initObj(Point2D<float> , vector<astar::sObs_t>&, vector<Obj*>& ) override {};
        int loopObj(std::vector<astar::sObs_t>& obs, std::vector<uint8_t>& obs_updated, vector<Obj*>&, std::vector<Actuator>&) override;
        eObj_t type() const override {return E_SPOT;};

    private :
        unsigned int _num;
        eColor_t _color;

};

#endif /* OBJ_CLAP_H_ */
