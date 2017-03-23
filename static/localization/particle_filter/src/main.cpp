#include <iostream>
#include <fstream>
#include "ParticleFilter.h"

using namespace std;
int main(){
	/*AcqLidar acqlidar = AcqLidar();
	if(acqlidar.init()) {
		perror("[LIDAR] Can't open serial.");
		return 1;
	}*/
	vector<float> measures = {5000., 50., 250., 5000., 200., 5000., 850., 5000., 5000., 1000.};
	ParticleFilter p = ParticleFilter();
	p.sense(measures);
	p.move(0., 0.);
	p.sense(measures);
	PointOrient2D<float> pto = p.locate();
	cout << "X : " << pto.p.x << " Y : " << pto.p.y << " Theta : " << pto.o;

}
