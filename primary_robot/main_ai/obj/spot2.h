/*
 * spot2.h
 *
 *  Created on: 9 mai 2015
 *      Author: seb
 */

#ifndef OBJ_SPOT2_H_
#define OBJ_SPOT2_H_

#include <types.h>
#include "obj.h"
#include "tools.h"

#define START_STAND2 4 //number of the first stand element in obs[]

using namespace std;

typedef enum{FIRST_ELEMENT, WAIT_GET_FIRST_SPOT, ROTATION, WAIT_ROTATION, SECOND_ELEMENT,WAIT_GET_SECOND_SPOT, END}  stepSpot2;

class Spot2 : public Obj{
    public:
        Spot2(unsigned int num, eColor_t color) : Obj(E_SPOT2, ActuatorType::ELEVATOR, true), rotAngle(0), _num(num), _state_loc(FIRST_ELEMENT){

            if(num > 2){
                logs << ERR << "Num too big";
            }

            sObjEntry_t objEP;

            _num_obs_loc[0] = START_STAND2;
            _num_obs_loc[1] = START_STAND2+1;

            if(num == 0){
                _posSpot[0] = {85. ,180.};
                _posSpot[1] = {85. ,190.};
                _posRobotSpot[0] = {75., 200 - 30.};
                _posRobotSpot[1] = {75., 200 - 22.};
                objEP.pt.angle = -M_PI/2;
            }
            else{
                _posSpot[0] = {9. ,25.};
                _posSpot[1] = {9. ,15.};
                _posRobotSpot[0] = {22.,  30. + 5.};
                _posRobotSpot[1] = {22.,  22. + 5.};
                objEP.pt.angle = M_PI/2;
                _num_obs_loc[0] += 2;
                _num_obs_loc[1] += 2;
            }

            Point2D<float> accessPoint[] = {{75, 160},{22., 45}};


            if(color == eColor_t::GREEN){
                accessPoint[0].x = 300. - accessPoint[0].x;
                accessPoint[1].x = 300. - accessPoint[1].x;
                _posSpot[0].x =  300. - _posSpot[0].x;
                _posSpot[1].x =  300. - _posSpot[1].x;
                _posRobotSpot[0].x =  300. - _posRobotSpot[0].x;
                _posRobotSpot[1].x =  300. - _posRobotSpot[1].x;
                _num_obs_loc[0] += 8;
                _num_obs_loc[1] += 8;
            }

            objEP.type = E_POINT;
            objEP.radius = 10.;
            objEP.pt.p = accessPoint[num];

            Vector2D<float> v1(accessPoint[num], _posSpot[0]), v2(0,1);
            objEP.delta = v2.angle(v1);

            _access.push_back(objEP);
        }
        virtual ~Spot2(){};

        void initObj(paramObj)  {}

        int loopObj(paramObj par) {

            switch(_state_loc){
                case FIRST_ELEMENT:
                    {
                        Vector2D<float> v1(_posRobotSpot[0], _posSpot[0]), v2(0,1);
                        path.go2PointOrient(_posRobotSpot[0], par.obs,
                                _access[0].pt.angle + M_PI + v2.angle(v1) + par.act[_actuator_select].angle);
                    }
                    _state_loc = WAIT_GET_FIRST_SPOT;
                    break;

                case WAIT_GET_FIRST_SPOT:
                    if(par.posRobot.distanceTo(_posRobotSpot[0]) < 1.){
                        //TODO servo elevator
                        _state_loc = ROTATION;
                    }
                    break;

                case ROTATION:
                    {
                        Vector2D<float> v3(_posRobotSpot[1], _posSpot[1]), v4(0,1);
                        rotAngle = _access[0].pt.angle + M_PI + v4.angle(v3) + par.act[_actuator_select].angle;
                        path.go2PointOrient(par.posRobot, par.obs, rotAngle);
                    }
                    _state_loc = WAIT_ROTATION;
                    break;

                case WAIT_ROTATION:
                    if(fabs(fmodf(rotAngle - par.angleRobot, 2*M_PI)) < M_PI/180.){
                        //TODO servo elevator
                        _state_loc = SECOND_ELEMENT;
                    }
                    break;

                case SECOND_ELEMENT:
                    {
                        Vector2D<float> v1(_posRobotSpot[1], _posSpot[1]), v2(0,1);
                        path.go2PointOrient(_posRobotSpot[1], par.obs,
                                _access[0].pt.angle + M_PI + v2.angle(v1) + par.act[_actuator_select].angle);
                    }
                    _state_loc = WAIT_GET_SECOND_SPOT;
                    break;
                case WAIT_GET_SECOND_SPOT:
                    if(par.posRobot.distanceTo(_posRobotSpot[1]) < 1.){
                        //TODO servo elevator
                        _state_loc = END;
                    }
                    break;

                case END:
                    _state = FINISH;
                    par.obs[_num_obs_loc[0]].active = 0;
                    par.obs[_num_obs_loc[1]].active = 0;
                    par.obsUpdated[_num_obs_loc[0]]++;
                    par.obsUpdated[_num_obs_loc[1]]++;

                    par.act[_actuator_select].elevator.number += 2;
                    par.act[_actuator_select].elevator.empty = false;

                    if(par.act[_actuator_select].elevator.number > 4){
                        logs << ERR << "No place in elevator";
                    }

                    if(par.act[_actuator_select].elevator.number == 4)
                        par.act[_actuator_select].elevator.full = true;

                    return 0;
                    break;
            }
            return 1;
        }

        eObj_t type() const  {
            return E_SPOT2;
        };

        int updateDestPointOrient(paramObj par){
            unsigned int i;

            if(par.act.empty())
                return -1;

            for(i = 0 ; i < par.act.size() ; i++){ //TODO optimize for the moment the first find is used
                if(par.act[i].type == _typeAct){
                    if((!par.act[i].elevator.full && par.act[i].elevator.number < 3))
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
        float rotAngle;
        unsigned int _num;
        stepSpot2 _state_loc;
        unsigned int _num_obs_loc[2];
        Point2D<float> _posSpot[2];
        Point2D<float> _posRobotSpot[2]; //position of the robot to get the spot

};



#endif /* OBJ_SPOT2_H_ */
