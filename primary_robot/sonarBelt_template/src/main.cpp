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
	SonarBelt* sonarBelt = new SonarBelt(1, initListSonars1, orderToProcess_3PerRev);
	cout << "Intanciated SonarBelt" << endl;


	int nbRevo = 0, i = 0;
	while(i < 20){
		nbRevo = sonarBelt->getNbRevo();
		for(listSonar_t::const_iterator it = initListSonars1.begin();
				it!=initListSonars1.end(); ++it){
			cout<<"SRF02: id = "<<it->first<<", last dist = "<<sonarBelt->getSonarDist(it->first)<<std::endl;
		}
		i++;
		std::cout<<" i = "<<i<<", nbRevo = "<<nbRevo<<std::endl;
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	}

	cout << "See you!!!" << endl;
	return 0;
}
