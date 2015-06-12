/*
 * light.h
 *
 *  Created on: 10 mai 2015
 *      Author: seb
 */

#ifndef OBJ_LIGHT_H_
#define OBJ_LIGHT_H_

#include <types.h>
#include "obj.h"
#include "tools.h"
#include "GeometryTools.h"


using namespace std;

typedef enum{ GO_TO_LIGHT, WAIT_LIGHT, BACK, END_LIGHT} lightStep;

class Light : public Obj{
    public:
        Light(eColor_t color) : Obj(E_LIGHT, ActuatorType::ELEVATOR, false), step(lightStep::GO_TO_LIGHT){
            Point2D<float> EP(55, 100); //Yellow
            sObjEntry_t objEP;

            if(color == eColor_t::GREEN)
                EP.x = 300 - EP.x;

            _state = ACTIVE;

            objEP.type = E_POINT;
            objEP.radius = 10;
            objEP.delta = 0;

            objEP.pt.p = EP;
            if(color == eColor_t::GREEN)
                objEP.pt.angle = M_PI;
            else
                objEP.pt.angle = 0;

            _access.push_back(objEP);
        }
        virtual ~Light(){}

        void initObj(paramObj par)  {
            destSelect = {8 + R_ROBOT, 100}; //3.5
            if(par.color == eColor_t::GREEN)
                destSelect.x = 300. - destSelect.x;

            path.go2PointOrient(destSelect, par.obs, _access_select_angle);

        }
        int loopObj(paramObj par) {
            switch(step){
                case lightStep::GO_TO_LIGHT:
                    //TODO preparation servo
                    step = lightStep::WAIT_LIGHT;
                    break;
                case lightStep::WAIT_LIGHT:
                    if(par.posRobot.distanceTo(destSelect) < 1.){
                        step = lightStep::BACK;
                    }
                    break;
                case lightStep::BACK:
                    destSelect = {30, 100};
                    if(par.color == eColor_t::GREEN)
                        destSelect.x = 300. - destSelect.x;

                    path.go2PointOrient(destSelect, par.obs, _access_select_angle);
                    step = lightStep::END_LIGHT;
                    break;
                case lightStep::END_LIGHT:
                    if(par.posRobot.distanceTo(destSelect) < 1.){
                        par.act[_actuator_select].elevator.ball = true;
                        _state = FINISH;
                        return 0;
                    }
            }
            return 1;

        }
        eObj_t type() const  {
            return E_LIGHT;
        }

    int updateDestPointOrient(paramObj par) {
        unsigned int i;

        if(par.act.empty())
            return -1;

        for(i = 0 ; i < par.act.size() ; i++){
            if( par.act[i].type == ELEVATOR){
                if(!par.act[i].elevator.ball && par.act[i].elevator.empty)
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
        Point2D<float> destSelect;
        lightStep step;



};






#endif /* OBJ_LIGHT_H_ */
