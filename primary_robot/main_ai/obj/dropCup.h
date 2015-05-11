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

            if(num == 0)
                _state = WAIT_FREE_ZONE;
            else
                _state = WAIT_MES;

            objEP.type = E_CIRCLE;
            objEP.delta = 0;
            if(color == YELLOW)
                objEP.cir.c = {listEP[num].x, listEP[num].y};
            else
                objEP.cir.c = {300 - listEP[num].x, listEP[num].y};
            objEP.cir.r = 10.;
            _access.push_back(objEP);


        }
        virtual ~DropCup(){}

        void initObj(paramObj par) override {
            for(Actuator& i : par.act){
                if(i.type == ActuatorType::CUP && i.id == _actuator_select){
                    Point2D<float> posCup(par.posRobot.tranform(i.angle + par.angleRobot, i.pos));
                    Circle2D<float> c(posCup, 30);
                    destPoint = c.project(par.posRobot);
                    path.go2PointOrient(destPoint, par.obs, _access_select_angle);

                    break;
                }
            }
        }
        int loopObj(paramObj par) override{
            if(destPoint.distanceTo(par.posRobot) < 1.){
                unsigned int i;
                Point2D<float> posActuator, posRobot(par.obs[0].c.x, par.obs[0].c.y);

                for(Actuator& i : par.act){
                    if(i.type == ActuatorType::CUP && i.id == _actuator_select){
                        i.cupActuator.full = false;
                        i.cupActuator.distributor = false;
                        posActuator = par.posRobot.tranform(i.angle + par.angleRobot, i.pos);
                        break;
                    }
                }

                for(i = START_CUP ; i < START_CUP + 5 ; i++){
                    if(!par.obs[i].active){
                        par.obs[i].active = 1;
                        par.obs[i].c = {posActuator.x, posActuator.y};
                        par.obs[i].r = 5 + R_ROBOT;
                        par.obsUpdated[i]++;
                        break;
                    }
                }
                if(i == START_CUP + 5)
                    logs << ERR << "Magic Cup ??";

                for(i = 0 ; i < par.act.size() ; i++)
                    if(par.act[i].type == ActuatorType::CUP && par.act[i].cupActuator.full)
                        break;

                if(i == par.act.size())
                    for(Obj* i : par.obj)
                        if((i->type() == E_DROP_CUP) && (i->state() == ACTIVE))
                            i->state() = WAIT_MES;

                _state = FINISH;
                return 0;
            }
            return 1;
        }
        eObj_t type() const override {
            return E_DROP_CUP;
        }

    public:
        int EP_selected;
        Point2D<float> destPoint;

};




#endif /* OBJ_DROPCUP_H_ */
