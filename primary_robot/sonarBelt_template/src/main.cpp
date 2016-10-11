//============================================================================
// Name        : Sonar.cpp
// Author      : Yoyo
// Version     :
// Copyright   : Your copyright notice
// Description : Manage the sonars on the primary robot
//============================================================================

#include <iostream>
#include "SonarBelt.h"
#include <chrono>
#include <thread>


using namespace std;

int main() {
	cout << "____ Starting Sonar process" << endl;

	listSonar_t initListSonar = initListSonars_2016; //initListSonars1, initList1Sonar
	orderToProcess_t orderToProc = orderToProcess_3PerRev; //orderToProcess_3PerRev, orderToProcess_1PerRev

	SonarBelt* sonarBelt = new SonarBelt(1, initListSonar, orderToProc);
	cout << "Intanciated SonarBelt" << endl;

	int nbRevo = 0, loop = 0;
	int attemps = 2;

	std::cout<<"\n____ Sequence 1: Get the distances of all sonars____\n";
	std::chrono::high_resolution_clock::time_point startTime;
	std::chrono::milliseconds dur_ms;

	while(loop < 1){
		startTime= std::chrono::high_resolution_clock::now();

	    // For each sonar print its distance
		for(listSonar_t::const_iterator it = initListSonar.begin();
				it!=initListSonar.end(); ++it){
			for(int v=0; v<attemps; v++){
				cout<<"SRF02: id = "<<it->first + 1<<", last dist = "<<sonarBelt->getSonarDist(it->first)<<std::endl;
				std::this_thread::sleep_for(std::chrono::milliseconds(500));
			}
			std::cout<<std::endl;
		}
		loop++;

		// Print info
		dur_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
				std::chrono::high_resolution_clock::now() - startTime);
		nbRevo = sonarBelt->getNbRevo();
		std::cout<<"loop = "<<loop<<", nbRevo = "<<nbRevo<<" in the loop of "<<
				dur_ms.count()<<"ms"<<std::endl;

		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	}

	std::cout<<"____ Sequence 2: Get the distances of all sonars in a specific direction ___\n";
	double theta = 45., // The robot moves in the direction theta = 45°
		   delta = 90.; // We want to check the angular sector arround the velocity vector
	sonarBelt->scanInThisDirect(theta, delta);

	cout << "\nSee you!!!" << endl;
	return 0;
}
