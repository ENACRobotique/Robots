/**
 * Project Untitled
 */


#ifndef _PROCESSLIDARDATA_H
#define _PROCESSLIDARDATA_H

#include <vector>
#include "PointOrient2D.h"
#include "PtLidar.h"
#include "Group.h"
#include "ObjectRef.h"

#define DIST_GROUP 100
#define RAD_TO_DEG 57.295779513082320876798154814105
#define PI 3.1415926535897932384626433832795

using namespace std;

class ProcessLidarData {
public: 
    
	ProcessLidarData(vector<ObjectRef*> objects);
    void process();

    vector< PointOrient2D<int> > getAdversaires();
    void makeGroups(vector<PtLidar> points);
    void correlateGroupsToObjects();
    int getObjectAzimut(ObjectRef);
    int getObjectDistance(ObjectRef);
    PointOrient2D<int> computePosition(pair<ObjectRef*,Group >, pair<ObjectRef*,Group >);

	const PointOrient2D<int>& getPosition() const {
		return position;
	}
	PointOrient2D<int> getPos();

	void setPosition(const PointOrient2D<int>& position) {
		this->position = position;
	}

	void affCorrelation();

private: 
    vector<Group> groups;
    vector<ObjectRef*> ref_objects;
    PointOrient2D<int> position;
    vector<pair<ObjectRef*,Group > > objGroupCorrelations;
};

#endif //_PROCESSLIDARDATA_H
