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

using namespace std;

class ProcessLidarData {
public: 
    
    /**
     * @param lastPos
     * @param points
     */
    void process(vector<PtLidar> points, PointOrient2D<int> lastPos);
    
    PointOrient2D<int> getPos();
    
    vector< PointOrient2D<int> > getAdversaires();
    void makeGroups(vector<PtLidar> points);
private: 
    vector<Group> groups;
    vector<ObjectRef *> objects;
    PointOrient2D<int> position;
};

#endif //_PROCESSLIDARDATA_H
