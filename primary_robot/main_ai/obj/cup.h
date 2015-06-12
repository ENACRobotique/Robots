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
#include "GeometryTools.h"

#define START_CUP   20 //number of the first stand element in obs[]

using namespace std;

typedef enum{
    CUP_DOWN_PINCE,
    CUP_OPEN_PINCE,
    CUP_TRAJ1,
    CUP_WAIT_TRAJ1,
    CUP_CLOSE,
    CUP_UP,
    CUP_END
}stepCup;

class Cup : public Obj{
    public:
        Cup(const unsigned int num, vector<astar::sObs_t>& obs);
        virtual ~Cup();

        void initObj(paramObj) ;
        int loopObj(paramObj) ;
        eObj_t type() const  {return E_CUP;};

        int updateDestPointOrient(paramObj par){
            unsigned int i;

            if(par.act.empty())
                return -1;

            for(i = 0 ; i < par.act.size() ; i++){ //TODO optimize for the moment the first find is used
                if( par.act[i].type == _typeAct){
                    if(!par.act[i].cupActuator.full)
                        break;
                }
            }

            if(i == par.act.size()){
                _actuator_select = -1;
                return -1;
            }

             _access_select_angle += par.act[i].angle;
             _actuator_select = par.act[i].id;

            return 0;
        }

    private :
        unsigned int _num;
        unsigned int _time;
        stepCup stepLoc;
        Point2D<float> destPoint;
};



#endif /* OBJ_CUP_H_ */
