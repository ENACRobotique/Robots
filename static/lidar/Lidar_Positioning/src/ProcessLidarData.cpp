/**
 * Project Untitled
 */


#include "ProcessLidarData.h"
#include <ostream>
/**
 * ProcessLidarData implementation
 */


/**
 * @param points
 * @param lastPos
 */
void ProcessLidarData::process(vector<PtLidar> points, PointOrient2D<int> lastPos) {

}

/**
 * @return Pos2D
 */
PointOrient2D<int> ProcessLidarData::getPos() {
	PointOrient2D<int> pos;
	return pos;
}

/**
 * @return vector<Pos2D>
 */
vector<PointOrient2D<int> > ProcessLidarData::getAdversaires() {
	vector<PointOrient2D<int> > posAdversaires;

	return posAdversaires;
}

void ProcessLidarData::makeGroups(vector<PtLidar> points) {
	int start=0;
	for(int i = 1; i < 360; i++){
		if(points[i].valid && (points[i].distance - points[i-1].distance) > DIST_GROUP) {
			start = i;
			break;
		}
	}

	Group current_group = Group();

	for(int i=start; i<360+start;i++){
		if( abs(points[i%360].distance - points[(i-1)%360].distance) > DIST_GROUP || !points[i%360].valid){
			current_group.computeParameters();
			if(current_group.getNbPoints() > 0) {
				groups.push_back(current_group);
				cout << current_group << endl;
			}
			current_group = Group();
		}
		if(points[i%360].valid) {
			current_group.appendPoint(&points[i%360]);
		}
	}
}
