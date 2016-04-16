/*
 * duneheap.h
 *
 *  Created on: 24 févr. 2016
 *      Author: guilhem
 */

#ifndef OBJ_DUNEHEAP_H_
#define OBJ_DUNEHEAP_H_

#include <types.h>
#include "obj.h"
#include "tools.h"
#include "GeometryTools.h"

using namespace std;

#define START_DUNEHEAP 17 //position of the obstacle associated in obs list (environment.cpp)

typedef enum {
	DUNEHEAP_PREP,
	DUNEHEAP_CONTACT,
	DUNEHEAP_SCRAPE,
	DUNEHEAP_CAPTURE,
	DUNEHEAP_PREEND,
	DUNEHEAP_END
}stepDuneHeap;

class DuneHeap : public Obj{
public:
	DuneHeap(unsigned int num);
	virtual ~DuneHeap();

    void initObj(paramObj) override;
    int loopObj(paramObj) override;
    eObj_t type() const override {return E_DUNEHEAP;};

private:
    unsigned int _num;
    unsigned int _time;

    stepDuneHeap stepLoc;
    Point2D<float> scratchPoint;
    float scratchAngle;

    Point2D<float> swallowPoint;
    float swallowAngle;

    Point2D<float> capturePoint;
    float captureAngle;
};

#endif /* OBJ_DUNEHEAP_H_ */
