/*
 * obj.hpp
 *
 *  Created on: 30 janv. 2015
 *      Author: seb
 */

#ifndef OBJ_OBJ_H_
#define OBJ_OBJ_H_

#include <ai_types.h>
#include "types.h"
#include <vector>

using namespace std;

typedef enum {E_POINT, E_CIRCLE, E_SEGMENT}eTypeEntry_t;

typedef struct {
    eTypeEntry_t type;      //type of access

#if NON_HOLONOMIC
    float radius;           //size of the 3 approach circles
#endif

    union{
        struct{
            sPt_t p;        //point
            float angle;    //approach angle between 0 and 2 M_PI
        }pt;

        struct{
            sPt_t c;        //enter of the circle
            float r;        //radius of the circle
        }cir;

        struct{
            sSeg_t s;       //segment
            bool dir;       //true if (p1;p2)^(p1;probot) > 0
        }seg;
    };
} sObjEntry_t;

typedef enum {
    E_NULL, E_CLAP, E_SPOT
} eObj_t;

typedef enum {
    ACTIVE, WAIT_MES, NO_TIME, FINISH
} eStateObj_t;

class Obj {
    public:
        Obj();
        Obj(eObj_t type);
        Obj(eObj_t type, vector<unsigned int> &numObs, vector<sObjEntry_t> &entryPoint);
        virtual ~Obj();

        virtual void initObj(){};
        virtual int loopObj(){return -1;};
        virtual eObj_t type() const {return E_NULL;} ;
        virtual float gain(){return _dist;};

        void addAccess(sObjEntry_t &access);

        sNum_t update(sPt_t posRobot);

        float getDist() const;
        sPath_t getPath() const;
        sPt_t getDestPoint() const;
        eStateObj_t getState() const;
        sNum_t getYield();

    protected:
        eObj_t _type;                       //objective type
        int _point;                         //point number of the objective
        eStateObj_t _state;                 //if the objective is used or not
        sPt_t _access_select;               //the closest access select
        sNum_t _dist;                       //distance robot-objective (the closest access)
        sNum_t _time;                       //time robot-objective (the closest access) TODO no compute for the moment
        sPath_t _path;                      //path robot-objective (the closest access)
        sNum_t _done;                       //probability than the objective has already been completed by another robot
        vector<unsigned int> _num_obs;      //obstacle number associate to the objective need to deactivate
        vector<sObjEntry_t> _access;        //list of access to reach the objective
};

#endif /* OBJ_OBJ_H_ */
