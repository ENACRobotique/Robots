/*
 * dropSpot.h
 *
 *  Created on: 10 mai 2015
 *      Author: Sebastien Malissard
 */

#ifndef OBJ_DROPSPOT_H_
#define OBJ_DROPSPOT_H_


#include <types.h>
#include "obj.h"
#include "tools.h"
#include "GeometryTools.h"
#include "spot.h"


using namespace std;

class DropSpot : public Obj{
    public:
        DropSpot(int num, eColor_t color) : Obj(E_DROP_SPOT, ActuatorType::CUP, false), EP_selected(-1){
            vector<Point2D<float>> listEP{{20, 100}, {115, 5}}; //Yellow
            sObjEntry_t objEP;

            _state = WAIT_MES;

            objEP.type = E_CIRCLE;
            objEP.delta = 0;
            if(color == eColor_t::GREEN)
                listEP[num].x = 300 - listEP[num].x;

            objEP.cir.c = listEP[num];
            objEP.cir.r = 10.;
            _access.push_back(objEP);
        }
        virtual ~DropSpot(){}

        void initObj(paramObj par) override {
            for(Actuator& i : par.act){
                if(i.type == ActuatorType::ELEVATOR && i.id == _actuator_select){
                    Point2D<float> posSpot(par.posRobot.tranform(i.angle + par.angleRobot, i.pos));
                    Circle2D<float> c(posSpot, 30);
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
                    if(i.type == ActuatorType::ELEVATOR && i.id == _actuator_select){
                        i.cupActuator.full = false;
                        i.cupActuator.distributor = false;
                        posActuator = par.posRobot.tranform(i.angle + par.angleRobot, i.pos);
                        break;
                    }
                }

                for(i = START_STAND ; i < START_STAND + 16 ; i++){
                    if(!par.obs[i].active){
                        par.obs[i].active = 1;
                        par.obs[i].c = {posActuator.x, posActuator.y};
                        par.obs[i].r = 5 + R_ROBOT;
                        par.obsUpdated[i]++;
                        break;
                    }
                }
                if(i == START_STAND + 16)
                    logs << ERR << "Magic Stand ??";

                for(i = 0 ; i < par.act.size() ; i++)
                    if(par.act[i].type == ActuatorType::ELEVATOR && par.act[i].id == _actuator_select)
                        break;

                par.act[i].elevator.ball = false;
                par.act[i].elevator.full = false;
                par.act[i].elevator.empty = true;
                par.act[i].elevator.number = 0;

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





#endif /* OBJ_DROPSPOT_H_ */
