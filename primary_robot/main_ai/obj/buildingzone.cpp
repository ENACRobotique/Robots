/*
 * buildingzone.cpp
 *
 *  Created on: 19 mars 2016
 *      Author: guilhem
 */

#include <buildingzone.h>

BuildingZone::BuildingZone(eColor_t color) : Obj(E_BUILDING_ZONE, ActuatorType::SANDDOOR, true), _step(BUILD_1_ARRIVAL), _color(color){

	_state = WAIT_MES;

	sObjEntry_t entryPoint;
	entryPoint.type = E_POINT;
	if (_color == eColor_t::GREEN){
		entryPoint.pt.p = {190.,80.};
		entryPoint.pt.angle = M_PI;
	}else{
		entryPoint.pt.p = {107.,117.};
		entryPoint.pt.angle = 0;
	}

	_access.push_back(entryPoint);

}

BuildingZone::~BuildingZone() {
	// TODO Auto-generated destructor stub
}

void BuildingZone::initObj(paramObj){

}

int BuildingZone::loopObj(paramObj par){

	switch (_step){
	case BUILD_1_ARRIVAL:
		path.go2PointOrient(Point2D<float>(173., 117.), par.obs, M_PI);
		_step = BUILD_1_DEPARTURE;
		break;
	case BUILD_1_DEPARTURE:
		if (par.posRobot.distanceTo(Point2D<float>(173., 117.))<=2.){
			servo.openDoor(servoName::DOOR_1_L);
			servo.openDoor(servoName::DOOR_1_R);

			path.go2PointOrient(Point2D<float>(193.,117.), par.obs, M_PI);

			_state = WAIT_MES;
			_step = BUILD_2_ARRIVAL;
			return 0;
		}
		break;
	case BUILD_2_ARRIVAL:
		logs << "LE TEST FONCTIONNE !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!";
		return 0;
	}
	return 1;
}

