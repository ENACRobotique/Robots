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

class Obj {
    public:
        Obj();
        Obj(eObj_t type);
        Obj(eObj_t type, vector<unsigned int> &numObs, vector<sObjPt_t> &entryPoint);
        virtual ~Obj();

        virtual void initObj(){};
        virtual int loopObj(){return -1;};
        virtual eObj_t type() const {return E_NULL;} ;
        virtual float gain(){return 0;};

        void setEP(sObjPt_t &pt);


        //TODO fonction update dist and path and time
        sNum_t update(sPt_t posRobot);
        void updateTime() {_time = _dist / SPEED;};
        sPath_t path() {
            return _path;
        }
        ;
        sObjPt_t entryPoint(int num) {return _entryPoint[num];};
        int EP() {return _EP;};
        sPt_t destPoint();

        bool active() {
            return _active;
        }
        ;
        sNum_t value();

    protected:
        eObj_t _type;                       //objective type
        eStateObj_t _state;                 //if the objective is used or not //TODO group with probability
        vector<unsigned int> _numObs;       //obstacle number associate to the objective
        sNum_t _dist;                       //distance robot-objective (the closest EntryPoint)
        sNum_t _time;                       //time robot-objective (the closest EntryPoint)
        sPath_t _path;                      //path robot-objective (the closest EntryPoint)
        int _point;                         //point number that the objective can save
        int _EP;                            //the closest EntryPoint select
        bool _active;                       //FALSE=objective finished TRUE=objective unfinished
        sNum_t _done;                       //probability than the objective has already been completed by another robot
        vector<sObjPt_t> _entryPoint;       //list of access point to reach the objective
};

extern void updateEndTraj(sNum_t theta, sPt_t *pt, sNum_t r);

#endif /* OBJ_OBJ_H_ */
