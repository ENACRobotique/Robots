/*
 * spot4.h
 *
 *  Created on: 12 mai 2015
 *      Author: Sebastien Malissard
 */

#ifndef OBJ_SPOT4_H_
#define OBJ_SPOT4_H_

#include <types.h>
#include "obj.h"
#include "tools.h"

#define START_STAND4 6 //number of the first stand element in obs[]

using namespace std;

#define RESOLUTION_ANGLE    (M_PI/180.) // (in rad)
#define RESOLUTION_POS      (1.)        // (in cm)

#define DELTA_X                         22.
#define SPOT4_POINT_ENTRY               45.
#define SPOT4_POINT_GET_FRIST_STAND     40.
#define SPOT4_POINT_GET_SECOND_STAND    30.
#define SPOT4_POINT_CLAP                20. //y
#define SPOT4_POINT_END                 50. //x


typedef enum {
    SPOT4_TRAJ1,
    SPOT4_WAIT_TRAJ1,
    SPOT4_GET_STAND1,
    SPOT4_ROT1,
    SPOT4_WAIT_ROT1,
    SPOT4_TRAJ2,
    SPOT4_WAIT_TRAJ2,
    SPOT4_GET_STAND2,
    SPOT4_TRAJ3,
    SPOT4_WAIT_TRAJ3,
    SPOT4_TRAJ4,
    SPOT4_WAIT_TRAJ4,
    SPOT4_END
} stepSpot4;

class Spot4 : public Obj{
    public:
        Spot4(paramObj par) : Obj(E_SPOT2, ActuatorType::ELEVATOR, true), color(par.color),angleSelect(0), rotAngle(0), _state_loc(SPOT4_TRAJ1), actClap(0), timePrev(0){
            sObjEntry_t objEP;
            Point2D<float> EP(DELTA_X, SPOT4_POINT_ENTRY);

            _state = ACTIVE;

            _num_obs_loc[0] = START_STAND4;
            _num_obs_loc[1] = START_STAND4+1;

            if(color == GREEN){
                _num_obs_loc[0] += 8;
                _num_obs_loc[1] += 8;
                EP.x = 300. - EP.x;
            }

            objEP.type = E_POINT;
            objEP.radius = 10.;
            objEP.pt.p = EP;
            updateDestPointOrient(par);

            Point2D<float> p1(color==YELLOW?DELTA_X:300.-DELTA_X, SPOT4_POINT_ENTRY),
                    p2(par.obs[_num_obs_loc[0]].c.x, par.obs[_num_obs_loc[0]].c.y);
            Vector2D<float> v1(p2, p1), v2(0, 1);

            angleSelect = M_PI/2 + M_PI + v2.angle(v1) + par.act[_actuator_select].angle;

            objEP.pt.angle = M_PI/2;
            objEP.delta = v2.angle(v1);

            _access.push_back(objEP);
        }
        virtual ~Spot4(){};

        void initObj(paramObj) override {}

        int loopObj(paramObj par) override{
            Vector2D<float> v2(0, 1);

            switch(_state_loc){
                case SPOT4_TRAJ1:
                    computeAngleSelect(SPOT4_POINT_GET_FRIST_STAND, 0, par);
                    setDestPoint(SPOT4_POINT_GET_FRIST_STAND);
                    path.go2PointOrient(destPoint, par.obs, angleSelect);
                    _state_loc = SPOT4_WAIT_TRAJ1;
                    break;

                case SPOT4_WAIT_TRAJ1:
                    if(par.posRobot.distanceTo(destPoint) < 1.){
                        servo.unlockElevator(_actuator_select);
                        servo.downElevator(_actuator_select);
                        timePrev = millis();
                        _state_loc = SPOT4_GET_STAND1;
                    }
                    break;

                case SPOT4_GET_STAND1:
                    if(millis() - timePrev > 500){
                        servo.lockElevator(_actuator_select);
                        servo.upElevator(_actuator_select);
                        _state_loc = SPOT4_ROT1;
                    }
                    break;

                case SPOT4_ROT1:
                    computeAngleSelect(par.posRobot.y, 1, par);
                    path.go2PointOrient(par.posRobot, par.obs, angleSelect);
                    _state_loc = SPOT4_WAIT_ROT1;
                    break;

                case SPOT4_WAIT_ROT1:
                    if(distTheta(par.angleRobot)){
                        _state_loc = SPOT4_TRAJ2;
                    }
                    break;

                case SPOT4_TRAJ2:
                    computeAngleSelect(SPOT4_POINT_GET_SECOND_STAND, 1, par);
                    setDestPoint(SPOT4_POINT_GET_SECOND_STAND);
                    path.go2PointOrient(destPoint, par.obs, angleSelect);
                    _state_loc = SPOT4_WAIT_TRAJ2;
                    break;
                case SPOT4_WAIT_TRAJ2:
                    if(par.posRobot.distanceTo(destPoint) < 1.){
                        servo.unlockElevator(_actuator_select);
                        servo.downElevator(_actuator_select);
                        timePrev = millis();
                        _state_loc = SPOT4_GET_STAND2;
                    }
                    break;

                case SPOT4_GET_STAND2:
                    if(millis() - timePrev > 500){
                        servo.lockElevator(_actuator_select);
                        servo.upElevator(_actuator_select);
                        _state_loc = SPOT4_TRAJ3;
                    }
                    break;

                case SPOT4_TRAJ3:
                    unsigned int i;
                    for(i = 0 ; i < par.act.size() ; i++){
                        if(par.act[i].type == POP_CORN_LOADER){
                            break;
                        }
                    }

                    actClap = i;

                    angleSelect = M_PI  + par.act[i].angle;
                    angleSelect += color==YELLOW?M_PI:0;
                    angleSelect += color==YELLOW?-M_PI/2:M_PI/2; //delta

                    setDestPoint(SPOT4_POINT_CLAP);
                    path.go2PointOrient(destPoint, par.obs, angleSelect);
                    _state_loc = SPOT4_WAIT_TRAJ3;

                    servo.openPopcornLoader(actClap);
                    break;

                case SPOT4_WAIT_TRAJ3:
                    if(par.posRobot.distanceTo(destPoint) < 1.){
                        _state_loc = SPOT4_TRAJ4;
                    }
                    break;

                case SPOT4_TRAJ4:
                    destPoint.x = color==YELLOW?SPOT4_POINT_END:300.-SPOT4_POINT_END;
                    destPoint.y = SPOT4_POINT_CLAP;
                    path.go2PointOrient(destPoint, par.obs, angleSelect);
                    _state_loc = SPOT4_WAIT_TRAJ4;
                    break;

                case SPOT4_WAIT_TRAJ4:
                    if(par.posRobot.distanceTo(destPoint) < 1.){
                        servo.closePopcornLoader(actClap);
                        _state_loc = SPOT4_END;
                    }
                    break;

                case SPOT4_END:
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

        eObj_t type() const override {
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

        void computeAngleSelect(float yi, unsigned int numObs, paramObj& par){
            Point2D<float> p1(color==YELLOW?DELTA_X:300.-DELTA_X, yi),
                    p2(par.obs[_num_obs_loc[numObs]].c.x, par.obs[_num_obs_loc[numObs]].c.y);
            Vector2D<float> v1(p2, p1), v2(0, 1);

            angleSelect = M_PI/2 + M_PI + v2.angle(v1) + par.act[_actuator_select].angle;
        }

        void setDestPoint(float y){
            destPoint.x = color==YELLOW?DELTA_X:300.-DELTA_X;
            destPoint.y = y;
        }

        bool distTheta(float angleRobot){
            float deltaTheta = fabs(fmod(angleRobot - angleSelect, 2*M_PI));

            if( deltaTheta < RESOLUTION_ANGLE || deltaTheta > (2*M_PI - RESOLUTION_ANGLE))
                return true;
            else
                return false;
        }

    private :
        eColor_t color;
        float angleSelect;
        float rotAngle;
        stepSpot4 _state_loc;
        unsigned int _num_obs_loc[2];
        unsigned int actClap;
        unsigned int timePrev;
        Point2D<float> destPoint;
        Point2D<float> _posSpot[2];
        Point2D<float> _posRobotSpot[2]; //position of the robot to get the spot

};




#endif /* OBJ_SPOT4_H_ */
