/*
 * objStartingZone.h
 *
 *  Created on: 10 mai 2015
 *      Author: seb
 */

#ifndef OBJ_OBJSTARTINGZONE_H_
#define OBJ_OBJSTARTINGZONE_H_


#include <types.h>
#include "obj.h"
#include "tools.h"
#include "GeometryTools.h"


using namespace std;

#define RESOLUTION_ANGLE    (M_PI/180.) // (in rad)
#define RESOLUTION_POS      (1.)        // (in cm)

#define POINT_DROP_FRIST_STAND      30. //and catch
#define POINT_DROP_SECOND_STAND     40. //and catch
#define POINT_ROTATION1             50.
#define POINT_GET_LIGHT             20.


typedef enum {
    GET_POS,
    TRAJ1,
    WAIT_TRAJ1,
    UNLOCK_STAND1,
    DROP_STAND1,
    TRAJ2,
    WAIT_TRAJ2,
    UNLOCK_STAND2,
    DROP_STAND2,
    TRAJ3,
    WAIT_TRAJ3,
    ROT1,
    WAIT_ROT1,
    TRAJ4,
    WAIT_TRAJ4,
    CATCH_STAND1,
    TRAJ5,
    WAIT_TRAJ5,
    CATCH_STAND2,
    ROT2,
    WAIT_ROT2,
    TRAJ6,
    WAIT_TRAJ6,
    CATCH_LIGHT,
    TRAJ7,
    WAIT_TRAJ7,
    ROT3,
    WAIT_ROT3,
    DROP_SPOT,
    TRAJ8,
    WAIT_TRAJ8,
    ROT4,
    WAIT_ROT4,
    DROP_CUP,
    TRAJ9,
    WAIT_TRAJ9,
    OSZ_END
} stepObjStratingZone;


class ObjStartingZone : public Obj{
    public:
        ObjStartingZone(eColor_t _color) : Obj(E_OBJ_STARTING_ZONE, ActuatorType::CAMERA, false),
                stateLoc(GET_POS), destPoint(0, 100), angleSelect(0),  color(_color), timePrev(0){
            Point2D<float> EP{50, 100}; //Yellow
            sObjEntry_t objEP;

            _state = WAIT_MES;

            objEP.type = E_POINT;
            objEP.radius = 15.;
            objEP.delta = M_PI;
            if(color == eColor_t::GREEN){
                EP.x = 300. - EP.x;
                objEP.pt.angle = M_PI;  //TODO dépend de la position de la caméra
            }
            else
                objEP.pt.angle = 0;     //TODO dépend de la position de la caméra
            objEP.pt.p = EP;

            _access.push_back(objEP);
        }
        virtual ~ObjStartingZone(){}

        void initObj(paramObj) override {}

        int loopObj(paramObj par) override{
            switch(stateLoc){
                case GET_POS:
                    stateLoc = TRAJ1;
                    break;

                case TRAJ1:
                    logs << WAR << "Start TRAJ1";
                    setDestPointX(POINT_DROP_FRIST_STAND);
                    setAngleSelect(par.act, ActuatorType::ELEVATOR, false);
                    path.go2PointOrient(destPoint, par.obs, angleSelect);
                    stateLoc = WAIT_TRAJ1;
                    break;

                case WAIT_TRAJ1:
                    if(par.posRobot.distanceTo(destPoint) < RESOLUTION_POS){
                        servo.downElevator(_actuator_select);
                        timePrev = millis();
                        stateLoc = UNLOCK_STAND1;
                    }
                    break;

                case UNLOCK_STAND1:
                    if(millis() - timePrev){
                        servo.unlockElevator(_actuator_select);
                        timePrev = millis();
                        stateLoc = DROP_STAND1;
                    }
                    break;

                case DROP_STAND1:
                    if(millis() - timePrev){
                        servo.upElevator(_actuator_select);
                        timePrev = millis();
                        stateLoc = TRAJ2;
                    }
                    break;

                case TRAJ2:
                    if(millis() - timePrev){
                        setDestPointX(POINT_DROP_SECOND_STAND);
                        path.go2PointOrient(destPoint, par.obs, angleSelect);
                        stateLoc = WAIT_TRAJ2;
                    }
                    break;

                case WAIT_TRAJ2:
                    if(par.posRobot.distanceTo(destPoint) < RESOLUTION_POS){
                        servo.downElevator(_actuator_select);
                        timePrev = millis();
                        stateLoc = UNLOCK_STAND2;
                    }
                    break;

                case UNLOCK_STAND2:
                    if(millis() - timePrev > 500.){
                        stateLoc = DROP_STAND2;
                    }
                    break;

                case DROP_STAND2:
                    stateLoc = TRAJ3;
                    break;

                case TRAJ3:
                    logs << WAR << "Start TRAJ3";
                    setDestPointX(POINT_ROTATION1);
                    path.go2PointOrient(destPoint, par.obs, angleSelect);
                    stateLoc = WAIT_TRAJ3;
                    break;

                case WAIT_TRAJ3:
                    if(par.posRobot.distanceTo(destPoint) < 1.){
                        stateLoc = ROT1;
                    }
                    break;
                case ROT1:
                    logs << WAR << "Start ROT1";
                    setAngleSelect(par.act, ActuatorType::ELEVATOR, true);
                    path.go2PointOrient(destPoint, par.obs, angleSelect);
                    stateLoc = WAIT_ROT1;
                    break;

                case WAIT_ROT1:
                    if(distTheta(par.angleRobot)){
                        stateLoc = TRAJ4;
                    }
                    break;

                case TRAJ4:
                    logs << WAR << "Start TRAJ4";
                    setDestPointX(POINT_DROP_SECOND_STAND);
                    path.go2PointOrient(destPoint, par.obs, angleSelect);
                    stateLoc = WAIT_TRAJ4;
                    break;
                case WAIT_TRAJ4:
                    if(par.posRobot.distanceTo(destPoint) < RESOLUTION_POS){
                        stateLoc = CATCH_STAND1;
                    }
                    break;
                case CATCH_STAND1:
                    logs << WAR << "Catch spot1";
                    stateLoc = TRAJ5;
                    break;

                case TRAJ5:
                    logs << WAR << "Start TRAJ5";
                    setDestPointX(POINT_DROP_FRIST_STAND);
                    path.go2PointOrient(destPoint, par.obs, angleSelect);
                    stateLoc = WAIT_TRAJ5;
                    break;

                case WAIT_TRAJ5:
                    if(par.posRobot.distanceTo(destPoint) < RESOLUTION_POS){
                        stateLoc = CATCH_STAND2;
                    }
                    break;

                case CATCH_STAND2:
                    logs << WAR << "Catch spot1";
                    stateLoc = ROT2;
                    break;

               case ROT2:
                    logs << WAR << "Start ROT2";
                    setAngleSelect(par.act, ActuatorType::ELEVATOR, false);
                    path.go2PointOrient(destPoint, par.obs, angleSelect);
                    stateLoc = WAIT_ROT2;
                    break;

                case WAIT_ROT2:
                    if(distTheta(par.angleRobot)){
                        stateLoc = TRAJ6;
                        }
                    break;

                case TRAJ6:
                    logs << WAR << "Start TRAJ6";
                    setDestPointX(POINT_GET_LIGHT);
                    path.go2PointOrient(destPoint, par.obs, angleSelect);
                    stateLoc = WAIT_TRAJ6;
                    break;

                case WAIT_TRAJ6:
                    if(par.posRobot.distanceTo(destPoint) < RESOLUTION_POS){
                        stateLoc = CATCH_LIGHT;
                    }
                    break;

                case CATCH_LIGHT:
                    logs << WAR << "Catch light";
                    stateLoc = TRAJ7;
                    break;

                case TRAJ7:
                    logs << WAR << "Start TRAJ7";
                    setDestPointX(POINT_DROP_FRIST_STAND);
                    path.go2PointOrient(destPoint, par.obs, angleSelect);
                    stateLoc = WAIT_TRAJ7;
                    break;

                case WAIT_TRAJ7:
                    if(par.posRobot.distanceTo(destPoint) < RESOLUTION_POS){
                        stateLoc = ROT3;
                    }
                    break;

               case ROT3:
                    logs << WAR << "Start ROT3";
                    setAngleSelect(par.act, ActuatorType::ELEVATOR, true);
                    path.go2PointOrient(destPoint, par.obs, angleSelect);
                    stateLoc = WAIT_ROT3;
                    break;

                case WAIT_ROT3:
                    if(distTheta(par.angleRobot)){
                        stateLoc = DROP_SPOT;
                    }
                    break;

                case DROP_SPOT:
                    stateLoc = TRAJ8;
                    break;

                case TRAJ8:
                    logs << WAR << "Start TRAJ8";
                    setDestPointX(POINT_DROP_SECOND_STAND);
                    path.go2PointOrient(destPoint, par.obs, angleSelect);
                    stateLoc = WAIT_TRAJ8;
                    break;

                case WAIT_TRAJ8:
                    if(par.posRobot.distanceTo(destPoint) < RESOLUTION_POS){
                        stateLoc = ROT4;
                    }
                    break;

               case ROT4:
                    logs << WAR << "Start ROT3";
                    setAngleSelect(par.act, ActuatorType::CUP, true);
                    path.go2PointOrient(destPoint, par.obs, angleSelect);
                    stateLoc = WAIT_ROT4;
                    break;

                case WAIT_ROT4:
                    if(distTheta(par.angleRobot)){
                        stateLoc = DROP_CUP;
                        }
                    break;

                case DROP_CUP:
                    stateLoc = TRAJ9;
                    break;

                case TRAJ9:
                    logs << WAR << "Start TRAJ9";
                    setDestPointX(POINT_ROTATION1);
                    path.go2PointOrient(destPoint, par.obs, angleSelect);
                    stateLoc = WAIT_TRAJ9;
                    break;

                case WAIT_TRAJ9:
                    if(par.posRobot.distanceTo(destPoint) < RESOLUTION_POS){
                        stateLoc = OSZ_END;
                    }
                    break;

                case OSZ_END:
                    for(unsigned int i = 0 ; i < par.act.size(); i++){
                        if(par.act[i].type == ActuatorType::ELEVATOR){
                            par.act[i].elevator.full = false;
                            par.act[i].elevator.empty = true;
                            par.act[i].elevator.number = 0;
                            par.act[i].elevator.ball = par.act[i].id==0?false:true;
                        }
                    }
                    _state = FINISH;
                    return 0;
                }

            return 1;
        }

        eObj_t type() const override {
            return E_OBJ_STARTING_ZONE;
        }

    private:
        void setDestPointX(float x){
            destPoint.x = color==eColor_t::PURPLE?x:300-x;
        }

        void setAngleSelect(std::vector<Actuator>& act, ActuatorType type, bool par){ //if elevator par=ball, if cupActuator par=full
            unsigned int i;
            float angleAct = 0;

            for(i = 0 ; i < act.size(); i++){
                if(act[i].type == type){
                    if(act[i].type == ELEVATOR){
                        if(act[i].elevator.ball == par){
                            angleAct = act[i].angle;
                            break;
                        }
                    }
                    else if(act[i].type == CUP){
                        if(act[i].elevator.ball == par){
                            angleAct = act[i].angle;
                            break;
                        }
                    }
                }
            }

            if(i == act.size()){
                logs << ERR << "No actuator found";
                return;
            }

            _actuator_select = i;

            angleSelect = M_PI + angleAct;
            angleSelect += color == eColor_t::GREEN?M_PI:0;

        }

        bool distTheta(float angleRobot){
            float deltaTheta = fabs(fmod(angleRobot - angleSelect, 2*M_PI));

            if( deltaTheta < RESOLUTION_ANGLE || deltaTheta > (2*M_PI - RESOLUTION_ANGLE))
                return true;
            else
                return false;
        }

        stepObjStratingZone stateLoc;
        Point2D<float> destPoint;
        float angleSelect;
        eColor_t color;
        unsigned int timePrev;



};





#endif /* OBJ_OBJSTARTINGZONE_H_ */
