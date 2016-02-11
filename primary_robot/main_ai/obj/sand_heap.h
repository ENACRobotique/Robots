/*
 * sand_heap.h
 *
 *  Created on: 11 janv. 2016
 *      Author: guilhem
 */

#ifndef OBJ_SAND_HEAP_H_
#define OBJ_SAND_HEAP_H_

#include <types.h>
#include "obj.h"
#include "tools.h"
#include "GeometryTools.h"

#define START_CUP   15 //number of the first stand element in obs[] (defined in environnement.cpp)

using namespace std;

typedef enum{
    SAND_HEAP_PREP,
    SAND_HEAP_PUSH,
	SAND_HEAP_BACK,
    SAND_HEAP_END
}stepSandHeap;

class SandHeap : public Obj{
    public:
        SandHeap(const unsigned int num);
        virtual ~SandHeap();

        void initObj(paramObj) override;
        int loopObj(paramObj) override;
        eObj_t type() const override {return E_SANDHEAP;};

        /*int updateDestPointOrient(paramObj par){
            unsigned int i;

            if(par.act.empty())
                return -1;

             _access_select_angle += par.act[i].angle;
             _actuator_select = par.act[i].id;

            return 0;
        }*/

    private :
        unsigned int _num;
        unsigned int _time;
        stepSandHeap stepLoc;
        Point2D<float> destPoint;
        Circle2D<float> backPoint;
        Circle2D<float> pointPutHeap;
};




#endif /* OBJ_SAND_HEAP_H_ */
