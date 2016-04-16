/**
 * Class file for Group
 */


#include "Group.h"
#include <ostream>

/**
 * Group implementation
 */


/**
 * @param dist
 * @param azimut
 * @return int
 */
void Group::appendPoint(PtLidar * pt) {
	points.push_back(pt);
}

/**
 * @return int
 */
int Group::computeParameters() {
	if(points.size() == 0){

		//cout << "[CALC PARAMS] : groupe sans point" << endl;
		return -1;
	}

	int a1 = points.front()->azimut;
	int a2 = points.back()->azimut;
	int offset = 0;
	if(a1 > a2){
		offset = 360;
	}
	azimut = ((a1 + a2 + offset)/2) % 360;


	int d1 = points.front()->distance;
	int d2 = points.back()->distance;

	double x1 = d1*cos(a1*DEG_TO_RAD);
	double y1 = d1*sin(a1*DEG_TO_RAD);
	double x2 = d2*cos(a2*DEG_TO_RAD);
	double y2 = d2*sin(a2*DEG_TO_RAD);
	sizeMin = (int) sqrt(pow((x2-x1),2) + pow((y2-y1),2));

	a1-=1;
	a2+=1;

	x1 = d1*cos(a1*DEG_TO_RAD);
	y1 = d1*sin(a1*DEG_TO_RAD);
	x2 = d2*cos(a2*DEG_TO_RAD);
	y2 = d2*sin(a2*DEG_TO_RAD);
	sizeMax = (int) sqrt(pow((x2-x1),2) + pow((y2-y1),2));

	distance = (d1 + d2) / 2;

	nbPoints = (int) points.size();

    return 0;
}

/**
 * @return int
 */
int Group::getAzimut() {
    return azimut;
}

/**
 * @return int
 */
int Group::getDist() {
    return distance;
}

/**
 * @return int
 */
int Group::getMinSize() {
    return sizeMin;
}

/**
 * @return int
 */
int Group::getMaxSize() {
    return sizeMax;
}

/**
 * @param Group other
 * @return int
 */
int Group::getDistOtherGroup(Group other) {
	int a2 = other.getAzimut();
	int d2 = other.getDist();

	double x1 = distance*cos(azimut*DEG_TO_RAD);
	double y1 = distance*sin(azimut*DEG_TO_RAD);
	double x2 = d2*cos(a2*DEG_TO_RAD);
	double y2 = d2*sin(a2*DEG_TO_RAD);
	int dist = (int) sqrt(pow((x2-x1),2) + pow((y2-y1),2));

	return dist;
}

/**
 * @return int
 */
int Group::getNbPoints() {
    return nbPoints;
}

/**
 * @param ostream &flux
 * @return void
 */
void Group::afficher(ostream &flux) const {
	flux << "azimut : " << azimut << "°\t" << sizeMin << " < Size < " << sizeMax << "\tdistance : " << distance << "\tnbPoints : " << nbPoints;
    return;
}

ostream &operator<<( ostream &flux, const Group& grp)
{
	grp.afficher(flux) ;
    return flux;
}
