#include <iostream>
#include <fstream>
#include "ParticleFilter.h"
#include "AcqLidar.h"
#include <chrono>

using namespace std;
using namespace std::chrono;
int main() {
	AcqLidar acqlidar = AcqLidar();
	if (acqlidar.init()) {
		perror("[LIDAR] Can't open serial.");
		return 1;
	}
	acqlidar.updateData();
	ParticleFilter p = ParticleFilter();
	auto time = high_resolution_clock::now();
	while (true) {
		acqlidar.updateData();
		if (duration_cast<milliseconds>(high_resolution_clock::now() - time).count()
				> 500) {
			vector<PtLidar> data = acqlidar.getData();
			vector<float> measures = vector<float>(360);
			cout << "{" << data[0].distance;
			for (unsigned int i = 0; i < data.size(); i++) {
				cout << "," << data[i].distance;
				if (data[i].distance == 0 || !data[i].valid) {
					measures.push_back(5000.0);
				} else {
					measures.push_back(data[i].distance);
				}
			}
			cout << "}";
			p.sense(measures);
			p.move(0, 0);
			PointOrient2D<float> pto = p.locate();
			cout << "X : " << pto.p.x << " Y : " << pto.p.y << " Theta : "
					<< pto.o << "\n";
			auto time = high_resolution_clock::now();
		}
	}
}
