/*
 * obj.hpp
 *
 *  Created on: 30 janv. 2015
 *      Author: seb
 */

#ifndef OBJ_OBJ_H_
#define OBJ_OBJ_H_

#include <ai_tools.h>
#include "types.h"
#include <vector>
#include "GeometryTools.h"
#include "a_star.h"
#include "obj.h"
#include "tools.h"

//#define DEBUG_OBJ

using namespace std;

typedef enum{
    SPOT,
    CLAP,
    POP_CORN
}objective;

typedef enum {
    E_NULL, E_CLAP, E_SPOT, E_SPOT2, E_SPOT3, E_LIGHT, E_CUP, E_DROP_CUP
} eObj_t;

typedef enum {
    ANY, CUP, ELEVATOR, POP_CORN_LOADER
}ActuatorType;

typedef struct {
        ActuatorType type;
        int id;         //id by type of actuator
        bool full;      //true if full
        bool empty;
        float angle;
        Point2D<float> pos;

        struct{
            unsigned int number;
            bool ball;
        }elevator;

        struct{
            bool distributor; //true if the cup was fill by a distributor
        }cupActuator;

        struct{
            bool direction; //direction of the opening servo
        }popCornLoader;

} Actuator;

typedef enum {E_POINT, E_CIRCLE, E_SEGMENT}eTypeEntry_t;

typedef struct {
    eTypeEntry_t type;      //type of access
    float radius;           //size of the 3 approach circles
    float delta;            //delta between the actuator and the approach angle;


    struct{
        Point2D<float> p;        //point
        float angle;    //approach angle between 0 and 2 M_PI
    }pt;

    struct{
            Point2D<float> c;        //enter of the circle
        float r;        //radius of the circle
    }cir;

    struct{
        Segment2D<float> s;       //segment
        bool dir;       //true if (p1;p2)^(p1;probot) > 0
    }seg;

} sObjEntry_t;

typedef enum {
    ACTIVE, WAIT_MES, NO_TIME, FINISH
} eStateObj_t;
class Obj ;
typedef struct{
        Point2D<float>&         posRobot;
        float&                  angleRobot;
        eColor_t                color;
        vector<astar::sObs_t>&  obs;
        vector<uint8_t>&        obsUpdated;
        vector<Obj*>&           obj;
        vector<Actuator>&       act;

}paramObj;

class Obj {
    public:
        Obj();
        Obj(eObj_t type, ActuatorType typeAct, bool get);
        Obj(eObj_t type, ActuatorType _typeAct, bool get, vector<unsigned int> &numObs, vector<sObjEntry_t> &entryPoint);
        virtual ~Obj();

        virtual void initObj(paramObj) = 0;
        virtual int loopObj(paramObj) = 0;
        virtual eObj_t type() const {return E_NULL;} ;
        virtual float gain(){return _dist;};

        void addAccess(sObjEntry_t &access);

        float update(const bool axle,  std::vector<astar::sObs_t>& obs, const int robot);
        virtual int updateDestPointOrient(paramObj par);


        float getDist() const;
        sPath_t getPath() const;
        Point2D<float> getDestPoint() const;
        float getDestPointOrient() const;
        eStateObj_t getState() const;
        float getYield(const unsigned int start_time);
        vector<unsigned int> getNumObs(){
            return _num_obs;
        }
        void print();

    protected:
        string objAccess(eTypeEntry_t access){
            switch(access){
                case E_POINT:
                    return "POINT";
                case E_CIRCLE:
                    return "CIRCLE";
                case E_SEGMENT:
                    return "SEGMENT";
                default:
                    return "Unknown type of access point";
            }
        }

        string objType(){
            switch(_type){
                case E_CLAP:
                    return "CLAP";
                case E_SPOT:
                    return "SPOT";
                case E_SPOT2:
                    return "SPOT2";
                case E_SPOT3:
                    return "SPOT3";
                case E_CUP:
                    return "CUP ";
                case E_DROP_CUP:
                    return "E_DROP_CUP";
                default:
                    return "Undefined";
            }
        }

        string objState(){
            switch(_state){
                case  ACTIVE:
                    return "activated";
                case WAIT_MES:
                    return "waiting a message";
                case NO_TIME:
                    return "no time to go to the goal";
                case FINISH:
                    return "finished";
                default:
                    return "Unknown state !!!";
            }
        }

        eObj_t _type;                       //objective type
        ActuatorType _typeAct;
        bool _get;                          //get or free object
        int _point;                         //point number of the objective
        eStateObj_t _state;                 //if the objective is used or not
        Point2D<float> _access_select;      //the closest access select
        float _access_select_angle;         //angle of the access select
        int _actuator_select;               //id of the actuator selected for this objective (link with the type of objective)
        float _dist;                        //distance robot-objective (the closest access)
        float _time;                        //time robot-objective (the closest access) TODO no compute for the moment
        sPath_t _path;                      //path robot-objective (the closest access)
        float _done;                        //probability than the objective has already been completed by another robot
        vector<unsigned int> _num_obs;      //obstacle number associate to the objective need to deactivate
    public:
        vector<sObjEntry_t> _access;        //list of access to reach the objective
        vector<sObjEntry_t>& access() { return _access; }        //list of access to reach the objective
        eStateObj_t& state() { return _state; }
};

#endif /* OBJ_OBJ_H_ */
