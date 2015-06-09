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

typedef enum{
    DROP_SPOT_INIT_WAIT,
    DROP_SPOT_INIT,
    DROP_SPOT_WAIT_ROT,
    DROP_SPOT_WAIT_CONTACT,
    DROP_SPOT_BACK,
    DROP_SPOT_UP_AND_CLOSE_DOOR,
    DROP_SPOT_END
}dropCupEnum;

class DropSpot : public Obj{
    public:
        DropSpot(int num, eColor_t color) : Obj(E_DROP_SPOT, ActuatorType::CUP, false), EP_selected(-1), step(DROP_SPOT_INIT_WAIT){
            vector<Point2D<float>> listEP{{160, 40}, {160, 40}}; //Yellow
            sObjEntry_t objEP;

            _state = WAIT_MES;

            objEP.type = E_CIRCLE;
            objEP.delta = M_PI;
            if(color == eColor_t::GREEN)
                listEP[num].x = 300 - listEP[num].x;

            objEP.cir.c = listEP[num];
            objEP.cir.r = 2.;
            _access.push_back(objEP);
        }
        virtual ~DropSpot(){}

        void initObj(paramObj /*par*/) override {
           /* for(Actuator& i : par.act){
                if(i.type == ActuatorType::ELEVATOR && i.id == _actuator_select){
                    Point2D<float> posSpot(par.posRobot.tranform(i.angle + par.angleRobot, i.pos));
                    Circle2D<float> c(posSpot, 30);
                    destPoint = c.project(par.posRobot);
                    path.go2PointOrient(destPoint, par.obs, _access_select_angle);

                    break;
                }
            }*/
        }

        int updateDestPointOrient(paramObj par){
            unsigned int i;

            if(par.act.empty())
                return -1;

            for(i = 0 ; i < par.act.size() ; i++){ //TODO optimize for the moment the first find is used
                if( par.act[i].type == _typeAct){
                    if((!par.act[i].elevator.full))
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

        int loopObj(paramObj par) override{
            _actuator_select=1;
            switch(step){
                case DROP_SPOT_INIT_WAIT:
                    _time = millis();
                    step = DROP_SPOT_INIT;

                    break;

                case DROP_SPOT_INIT:
                    if(millis() - _time > 1000){
                        destPoint.x = par.posRobot.x;
                        destPoint.y = 35.;
                        _access_select_angle = -(90.-48.)*M_PI/180;
                        path.go2PointOrient(destPoint, par.obs, _access_select_angle);
                        step = DROP_SPOT_WAIT_ROT;
                    }
                    break;

                case DROP_SPOT_WAIT_ROT:
                    if(destPoint.distanceTo(par.posRobot) < 0.2){
                        destPoint.x = par.posRobot.x;
                        destPoint.y = 15.;
                        path.go2PointOrient(destPoint, par.obs, _access_select_angle);
                        step = DROP_SPOT_WAIT_CONTACT;
                    }
                    break;

                case DROP_SPOT_WAIT_CONTACT:
                    if(destPoint.distanceTo(par.posRobot) < 1.){
                        Point2D<float> pos(par.posRobot);
                        pos.y = 10+11.2;
                        path.stopRobot(true);
                        sendMixPosPrimary(pos, _access_select_angle, MAXVARIANCE_XY, MINVARIANCE_XY, 0., MINVARIANCE_THETA);
                        step = DROP_SPOT_BACK;
                    }
                    break;

                case DROP_SPOT_BACK:
                    if(dropStand(_actuator_select)){
                        destPoint.x = par.posRobot.x;
                        destPoint.y = 40.;
                        path.go2PointOrient(destPoint, par.obs, _access_select_angle);
                        step = DROP_SPOT_UP_AND_CLOSE_DOOR;
                    }
                    break;

                case DROP_SPOT_UP_AND_CLOSE_DOOR:
                    if(destPoint.distanceTo(par.posRobot) < 1.){
                        servo.upElevator(_actuator_select);
                        servo.closeDoorElevator(_actuator_select);
                        _time = millis();
                        step = DROP_SPOT_END;
                    }
                    break;

                case DROP_SPOT_END:
                    if(millis() - _time > 100){
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
                               par.obs[i].c = {destPoint.x, 10};
                               par.obs[i].r = 10 + R_ROBOT;
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
                    break;

            }
            return 1;
        }

        eObj_t type() const override {
            return E_DROP_CUP;
        }

    public:
        int EP_selected;
        Point2D<float> destPoint;
        dropCupEnum step;

};





#endif /* OBJ_DROPSPOT_H_ */
