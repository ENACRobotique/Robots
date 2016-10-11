/*
 * buildingzone.h
 *
 *  Created on: 19 mars 2016
 *      Author: guilhem
 */

#ifndef OBJ_BUILDINGZONE_H_
#define OBJ_BUILDINGZONE_H_

#include <types.h>
#include "obj.h"
#include "tools.h"
#include "GeometryTools.h"


using namespace std;

typedef enum{
	BUILD_1_ARRIVAL,
	BUILD_1_DEPARTURE,
	BUILD_2_ARRIVAL

}stepBuildingZone;

class BuildingZone : public Obj{
public:
	BuildingZone(eColor_t color);
	virtual ~BuildingZone();
	void initObj(paramObj);
	int loopObj(paramObj);
	eObj_t type() const override {return E_BUILDING_ZONE;};

private:
	stepBuildingZone _step;
	eColor_t _color;

};

#endif /* OBJ_BUILDINGZONE_H_ */
