/**
 * Project Untitled
 */


#include "ProcessLidarData.h"
#include <ostream>
#include "math.h"
/**
 * ProcessLidarData implementation
 */
#define DELTA_DIST_MAX 400
#define DELTA_AZIMUT_MAX 25

/**
 * todo: Toute la fonction !
 * @param points
 * @param lastPos
 */
ProcessLidarData::ProcessLidarData(vector<ObjectRef*> objects) {
	ref_objects = objects;
}

void ProcessLidarData::process() {
	vector<PointOrient2D<int> > positions;
	int factor = 0;
	PointOrient2D<int>  _pos(0,0,0);
	for(unsigned int i = 0; i<objGroupCorrelations.size(); i++) {
		for(unsigned int j = i+1; j<objGroupCorrelations.size(); j++) {
			PointOrient2D<int>  pos = computePosition(objGroupCorrelations[i],objGroupCorrelations[j]);
			positions.push_back(pos);
			//_pos += objGroupCorrelations[i].second.getNbPoints() * objGroupCorrelations[j].second.getNbPoints() * pos;
			factor += objGroupCorrelations[i].second.getNbPoints() * objGroupCorrelations[j].second.getNbPoints();
		}
	}
	//_pos /= factor;
	//todo choisir la meilleure position.
}

PointOrient2D<int> ProcessLidarData::computePosition(pair<ObjectRef*,Group > pa, pair<ObjectRef*,Group > pb) {

	double rc = pow(pa.second.getDist(),2);
	double Rc = pow(pb.second.getDist(),2);
	double a = 2*(pb.first->posX - pa.first->posX);
	double b = 2*(pb.first->posY - pa.first->posY);
	double c = pow((pb.first->posX - pa.first->posX),2) + pow((pb.first->posY - pa.first->posY),2) - Rc + rc;
	double delta = pow(2*a*c,2) - 4*(a*a+b*b)*(c*c+b*b*rc);

	double xp = pa.first->posX + (2*a*c-sqrt(delta))/(2*(a*a+b*b));
	double xq = pa.first->posX + (2*a*c+sqrt(delta))/(2*(a*a+b*b));

	double yp,yq;
	if(abs(b) > 0.01) {  //b != 0
		yp = pa.first->posY + (c-a*(xp-pa.first->posX))/b;
		yq = pa.first->posY + (c-a*(xq-pa.first->posX))/b;
		cout << "a: " << a << "\tb: " << b << "\tc: " << c << "\tdelta" << delta << endl;
		cout << "xp: " << xp << "\typ: " << yp << "\txq: " << xq << "\tyq: " << yq <<endl;
	}
	else {
		cout << "on verra plus tard hein ?" << endl;	///TODO cf document pdf quand b=0
	}


	PointOrient2D<int> aze(0,0,0);
	return aze;
}

/**
 * todo: Toute la fonction !
 * @return Pos2D
 */
PointOrient2D<int> ProcessLidarData::getPos() {
	PointOrient2D<int> pos;
	return pos;
}

/**
 * todo: Toute la fonction !
 * @return vector<Pos2D>
 */
vector<PointOrient2D<int> > ProcessLidarData::getAdversaires() {
	vector<PointOrient2D<int> > posAdversaires;

	return posAdversaires;
}

/**
 *
 * @param points
 */
void ProcessLidarData::makeGroups(vector<PtLidar> points) {
	groups.clear();
	int start=0;
	for(int i = 1; i < 360; i++){
		if(points[i].valid && (points[i].distance - points[i-1].distance) > DIST_GROUP) {
			start = i;
			break;
		}
	}

	Group current_group = Group();

	for(int i=start; i<360+start;i++){
		//if( abs(points[i%360].distance - points[(i-1)%360].distance) > DIST_GROUP || !points[i%360].valid){
		if( abs(points[i%360].distance - points[(i-1)%360].distance)/(float)points[i%360].distance > 0.1 || !points[i%360].valid){
			current_group.computeParameters();
			if(current_group.getNbPoints() > 0) {
				groups.push_back(current_group);
				//cout << current_group << endl;
			}
			current_group = Group();
		}
		if(points[i%360].valid) {
			current_group.appendPoint(&points[i%360]);
		}
	}
}

/**
 * \brief Find correlation between groups detected and past known objects
 * \description 1) sort on expected size
 * 				2) sort on
 */
void ProcessLidarData::correlateGroupsToObjects() {
	objGroupCorrelations.clear();

	/*for(Group grp : groups) {
		cout << grp << endl;
	}
	for(ObjectRef* obj : ref_objects) {
		cout << obj->name << "\taz: " << (getObjectAzimut(*obj)-position.getO()+360)%360 << "\tdist: " << getObjectDistance(*obj) << endl;
	}*/

	for(ObjectRef* obj : ref_objects) {
		//cout << "Obj : " << obj->name << "grps :" << endl;
		for(Group grp : groups) {
			if(abs(getObjectDistance(*obj) - grp.getDist()) > DELTA_DIST_MAX) {
				continue;	//eliminate groups too far as expected
			}
			//cout << "[A] " << obj->name << "\taz: " << getObjectAzimut(*obj) << "\tdist: " << getObjectDistance(*obj) << "\tGRP : " << grp<< endl;
			if(abs( (getObjectAzimut(*obj)-position.getO())%360 - grp.getAzimut()) > DELTA_AZIMUT_MAX) {
				continue;	//eliminate groups with too different azimut
			}
			//cout << "[BB] " << obj->name << "\taz: " << getObjectAzimut(*obj) << "\tdist: " << getObjectDistance(*obj) << "\tGRP : " << grp<< endl;
			if(obj->size > grp.getMaxSize() || obj->size < grp.getMinSize()) {
				continue;	//eliminate groups with different size
			}

			//OK, this group is a candidate for the object
			//cout << "[CCC] " << obj->name << "\taz: " << getObjectAzimut(*obj) << "\tdist: " << getObjectDistance(*obj) << "\tsize: " << obj->size << "\tGRP : " << grp<< endl;
			objGroupCorrelations.push_back(make_pair(obj,grp));
		}

	}
}

void ProcessLidarData::affCorrelation() {
	for(pair<ObjectRef*,Group > pp : objGroupCorrelations) {
		cout << pp.first->name << ".\tgrps :" << pp.second << endl;
	}
}

/**
 * \brief Compute the azimut of the object in the robot referential.
 * @param obj
 * @return
 */
int ProcessLidarData::getObjectAzimut(ObjectRef obj) {
	double dx = obj.posX - position.p.getX();
	double dy = obj.posY - position.p.getY();
	int azimutObj = ((int)(atan2(dy,dx)*RAD_TO_DEG)+360)%360;

	//cout << obj.name << "\tdx: " << dx <<"\tdy: " << dy <<endl;

	return azimutObj;
}

/**
 * \brief Compute the distance of the object from the robot.
 * @param obj
 * @return
 */
int ProcessLidarData::getObjectDistance(ObjectRef obj) {
	double dx = obj.posX - position.p.getX();
	double dy = obj.posY - position.p.getY();
	int dist = sqrt(pow(dx,2) + pow(dy,2));
	return dist;
}
