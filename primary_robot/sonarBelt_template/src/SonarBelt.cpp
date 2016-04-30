/*!
 * @file SonarBelt.cpp
 * @brief Object with manage the belt of sonars and provide
 * information to the user like the distances measured by each sonar
 * @date Mar 26, 2016
 * @author yoann.solana@gmail.com
 */

#include "SonarBelt.h"


/**
 * @param idI2C int value which describe the i2c interface (i2c-0; i2c-1)
 * @param initListSonars
 * @param order
 * @param openI2C
 * @param file
 */
SonarBelt::SonarBelt(int idI2C, const listSonar_t initListSonars, const orderToProcess_t order) {
	_nbSonars = (int) initListSonars.size();
	std::cout<<"InitListSonar has a size of "<<initListSonars.size()<<std::endl;


	_nbRevolu = 0;
	_orderToProccess = order;

	// Create the sonars
	std::cout<<"Initialize list of SRF02: starting\n";
	for(listSonar_t::const_iterator it=initListSonars.begin();
			it!= initListSonars.end(); ++it){
		std::cout<<"SRF02 start initialization: id = "<<it->first<<", address = "<<it->second.first<<
				", orient = "<<it->second.second.first<<", r = "<<it->second.second.second<<std::endl;
		_sonars[it->first] = new SRF02(it->second.first, it->second.second);
		std::cout<<"SRF02 initialized: id = "<<it->first<<", address = "<<it->second.first<<std::endl;
	}
	std::cout<<"Initialize list of SRF02: finished and succeed\n";

	// Test connections with the sonars
#if defined(DBG)  &&  defined(DBG_SRF02)
	std::cout<<"_____ Test connection with sonars: start\n";
	for(mapSonars_t::iterator it = _sonars.begin(); it!= _sonars.end(); ++it){
		std::cout<<"sonar "<<it->first<<", addr = "<<it->second->getAddr()<<
				": vers = "<<it->second->readSRF02_info(srf02_version)<<std::endl;
	}
	std::cout<<"_____ Test connection with sonars: end\n";
#endif

	// Initialize thread for permanent measures
	try{
		_thMeasure = new std::thread(&SonarBelt::threadMeasure, this);
	}catch (std::exception &e) {
		std::cout<<e.what()<<std::endl;
	}
}

void SonarBelt::threadMeasure(){
	while(1){
		doMeasure_revol();
	}
}


int SonarBelt::getNbRevo(){
	std::cout<<"getNbRevo(): in"<<std::endl;
	_m_data.lock();
	int nb = _nbRevolu;
	_m_data.unlock();

	return nb;
}

void SonarBelt::writeSonarCmd(eIdSonar id, eSRF02_Cmd typeCmd){
	// Send the cmd to the sonar
	setCmdSonar(id, typeCmd);
}


int SonarBelt::getSonarDist(eIdSonar id){
	return _sonars.find(id)->second->getLastDist();
}

int SonarBelt::getSonarTimeIdx(eIdSonar id){
	return _sonars.find(id)->second->getTimeIdxLastDist();
}

int SonarBelt::getInfoSonar(eIdSonar id, eSRF02_Info infoType){
	return _sonars.find(id)->second->readSRF02_info(infoType);
}

void SonarBelt::setCmdSonar(eIdSonar id, eSRF02_Cmd cmdType){
	_sonars.find(id)->second->writeSRF02_cmd(cmdType);
}

void SonarBelt::launchBurst(eIdSonar id){
	writeSonarCmd(id, srf02_burst);
}

void SonarBelt::doMeasure_revol(){
	std::chrono::high_resolution_clock::time_point startTimeRev;
	std::chrono::duration<float> dur_s;
	std::chrono::milliseconds dur_ms;

	// Sent the command to start the measures for all the sonars
	for(int i=0; i<(int)_orderToProccess.size(); i++){  // for a revolution
		for(int j=0; j<(int)_orderToProccess[0].size(); j++){  // for a sonar on a revo
			eIdSonar id = _orderToProccess[i][j];
			writeSonarCmd(id, srf02_mes_cm);

			if(j==0)
				startTimeRev = std::chrono::high_resolution_clock::now();
		}
	}
	dur_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
			std::chrono::high_resolution_clock::now() - startTimeRev);

	// Wait the specific time and read the distance for each sonar
	while(dur_ms.count() < SRF02_MEAS_PERIOD){
//		std::cout<<"Unvalid dur = "<<dur_ms.count()<<" ms\n";  // Just to evaluate the method
		dur_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
				std::chrono::high_resolution_clock::now() - startTimeRev);
	}

	for(int i=0; i<(int)_orderToProccess.size(); i++){  // for a revolution
		for(int j=0; j<(int)_orderToProccess[0].size(); j++){  // for a sonar on a revo
			eIdSonar id = _orderToProccess[i][j];

			std::unique_lock<std::mutex> lk(_m_data);
			_sonars.find(id)->second->updateLastDist(_sonars.find(id)->second->readSRF02_info(srf02_range));
			_sonars.find(id)->second->updateTimeIdx(_nbRevolu);
			_nbRevolu++;
			lk.unlock();
		}
	}
}


//// For debug purpose
#ifdef DBG

#endif
