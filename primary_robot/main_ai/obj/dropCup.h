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
#include "servoTools.h"

extern "C"{
#include "millis.h"
}

typedef enum {
    DROP_CUP_DROP,
    CROP_CUP_OPEN,
    DROP_CUP_BACK,
    DROP_CUP_IN,
    DROP_CUP_END
}StepdropCup;

using namespace std;

class DropCup : public Obj{
    public:
        DropCup(int num, eColor_t color) : Obj(E_DROP_CUP, ActuatorType::CUP, false), EP_selected(-1), step(DROP_CUP_DROP), _time(0){
            vector<Point2D<float>> listEP{{50, 100}, {280, 50}, {280, 150}}; //Yellow
            sObjEntry_t objEP;

            _state = WAIT_MES;

            objEP.type = E_CIRCLE;
            objEP.delta = 0;
            if(color == eColor_t::YELLOW)
                objEP.cir.c = {listEP[num].x, listEP[num].y};
            else
                objEP.cir.c = {300 - listEP[num].x, listEP[num].y};
            objEP.cir.r = 10.;
            _access.push_back(objEP);


        }
        virtual ~DropCup(){}

        int updateDestPointOrient(paramObj par){
            unsigned int i;

            if(par.act.empty())
                return -1;

            for(i = 0 ; i < par.act.size() ; i++){ //TODO optimize for the moment the first find is used
                if( par.act[i].type == _typeAct){
                    if((par.act[i].cupActuator.full))
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

        void initObj(paramObj) override {
        }
        int loopObj(paramObj par) override{

            switch(step){
                case DROP_CUP_DROP:
                    if(dropCup(_actuator_select)){
                        for(Actuator& i : par.act){
                            if(i.type == ActuatorType::CUP && i.id == _actuator_select){
                                //Point2D<float> posCup(par.posRobot.tranform(par.angleRobot-i.angle, i.pos));
                                Point2D<float> posCup(_access[0].cir.c);
                                Circle2D<float> c(posCup, 30);
                                logs << ERR << _access_select;
                                destPoint = c.project(par.posRobot);
                                path.go2PointOrient(destPoint, par.obs, _access_select_angle);
                                _time = millis();
                                break;
                            }
                        }
                        step = CROP_CUP_OPEN;
                    }
                    break;

                case CROP_CUP_OPEN:
                    if(millis() - _time > 100){
                        servo.unlockPince(_actuator_select);
                        _time = millis();
                        step = DROP_CUP_BACK;
                    }
                    break;

                case DROP_CUP_BACK:
                    if(destPoint.distanceTo(par.posRobot) < 1.){
                        servo.inPince(_actuator_select);
                        _time = millis();
                        step = DROP_CUP_IN;
                    }
                    break;

                case DROP_CUP_IN:
                    if(millis() - _time > 1000){
                        servo.upPince(_actuator_select);
                        step = DROP_CUP_END;
                    }
                    break;

                case DROP_CUP_END:

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
                           // par.obs[i].c = {posActuator.x, posActuator.y};
                            par.obs[i].c = {_access[0].cir.c.x, _access[0].cir.c.y};
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
        StepdropCup step;
        unsigned int _time;

};




#endif /* OBJ_DROPCUP_H_ */
