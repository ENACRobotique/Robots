/*
 * spot3.h
 *
 *  Created on: 10 mai 2015
 *      Author: Sebastien Malissard
 */

#ifndef OBJ_SPOT3_H_
#define OBJ_SPOT3_H_


#include <types.h>
#include "obj.h"
#include "tools.h"
#include "GeometryTools.h"

#define START_STAND3    8

using namespace std;

class Spot3 : public Obj{
    public:
        Spot3(vector<astar::sObs_t>& obs, eColor_t color) : Obj(E_SPOT3, ActuatorType::ELEVATOR, true){
            Point2D<float> EP(30, 160); //Yellow
            sObjEntry_t objEP;
            unsigned int num_obs = START_STAND3;

            if(color == GREEN){
                EP.x = 300. - EP.x;
                num_obs += 8;
            }

            objEP.type = E_POINT;
            objEP.radius = 10.;
            objEP.delta = 0;

            objEP.pt.p = EP;
            Vector2D<float> v1({obs[num_obs].c.x,obs[num_obs].c.y}, EP), v2(1,0);
            logs << WAR << "EP.x" << EP.x << "EP.y" << EP.y;
            logs << WAR << "obs.x=" << obs[num_obs].c.x << "obs.y=" << obs[num_obs].c.y;
            objEP.pt.angle = v2.angle(v1);
logs << ERR << "angle =" <<  objEP.pt.angle*180/M_PI;
//objEP.cir.c.xgetchar();
            _access.push_back(objEP);
            _num_obs.push_back(num_obs);

            _state = ACTIVE;

        }
        virtual ~Spot3(){}

        void initObj(paramObj par) override {
            Point2D<float> posSpot(par.obs[_num_obs[0]].c.x, par.obs[_num_obs[0]].c.y);
            Circle2D<float> c(posSpot, 15.);
            destPoint = c.project(par.posRobot);
            path.go2PointOrient(destPoint, par.obs, _access_select_angle);
        }

        int loopObj(paramObj par) override{
            if(par.posRobot.distanceTo(destPoint) < 1.){
                //TODO servo elevator
                _state = FINISH;
                return 0;
            }
            return 1;
        }

        eObj_t type() const override {
            return E_SPOT3;
        }

    private:
        Point2D<float> destPoint;

};


#endif /* OBJ_SPOT3_H_ */
