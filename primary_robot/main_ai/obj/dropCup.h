/*
 * dropCup.h
 *
 *  Created on: 4 mai 2015
 *      Author: Sebastien Malissard
 */

#ifndef OBJ_DROPCUP_H_
#define OBJ_DROPCUP_H_


#include <types.h>
#include "obj.h"
#include "tools.h"
#include "GeometryTools.h"


using namespace std;

class DropCup : public Obj{
    public:
        DropCup(int num, eColor_t color) : Obj(E_DROP_CUP, ActuatorType::CUP, false), EP_selected(-1){
            vector<Point2D<float>> listEP{{50, 100}, {280, 50}, {280, 150}}; //Yellow
            sObjEntry_t objEP;

            _state = WAIT_MES;

            objEP.type = E_CIRCLE;
            if(color == YELLOW)
                objEP.cir.c = {listEP[num].x, listEP[num].y};
            else
                objEP.cir.c = {300 - listEP[num].x, listEP[num].y};
            objEP.cir.r = 10.;
            _access.push_back(objEP);


        }
        virtual ~DropCup(){}

        void initObj(Point2D<float> , vector<astar::sObs_t>&, vector<Obj*>&) override {

        }
        int loopObj(std::vector<astar::sObs_t>& listObs, std::vector<uint8_t>& obsUpdated, vector<Obj*>& listObj, std::vector<Actuator>& actuator) override{
            unsigned int i;

            for(Actuator& i : actuator){
                if(i.type == ActuatorType::CUP && i.id == _actuator_select){
                    i.full = false;
                    i.cupActuator.distributor = false;
                }
            }

            for(i = START_CUP ; i < START_CUP + 5 ; i++){
                if(!listObs[i].active){
                    listObs[i].active = 1;
                    listObs[i].c = {_access[0].cir.c.x, _access[0].cir.c.y};
                    listObs[i].r = 5 + R_ROBOT;
                    obsUpdated[i]++;
                    break;
                }
            }
            if(i == START_CUP + 5)
                logs << ERR << "Magic Cup ??";

            for(i = 0 ; i < actuator.size() ; i++)
                if(actuator[i].type == ActuatorType::CUP && actuator[i].full)
                    break;

            if(i == actuator.size())
                for(Obj* i : listObj)
                    if((i->type() == E_DROP_CUP) && (i->state() == ACTIVE))
                        i->state() = WAIT_MES;

            _state = FINISH;
            return 0;
        }
        eObj_t type() const override {
            return E_DROP_CUP;
        }

    public:
        int EP_selected;

};




#endif /* OBJ_DROPCUP_H_ */
