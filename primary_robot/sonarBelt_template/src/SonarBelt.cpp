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
	_nbRevolu = 0;
	_stateThreadMeas = notLaunched;
	_orderToProccess = order;

	// Create the sonars
	for(listSonar_t::const_iterator it=initListSonars.begin();
			it!= initListSonars.end(); ++it){
		_sonars[it->first] = new SRF02(it->second.first, it->second.second);
	}

	// Test connections with the sonars
#if defined(DBG)  &&  defined(DBG_SRF02)
	for(mapSonars_t::iterator it = _sonars.begin(); it!= _sonars.end(); ++it){
		std::cout<<"sonar "<<it->first<<", addr = "<<it->second->getAddr()<<
				": vers = "<<it->second->readSRF02_info(srf02_version)<<std::endl;
	}
#endif

	// Initialize thread for permanent measures
	_autoMeasure = false;
	try{
		_thMeasure = new std::thread(&SonarBelt::threadMeasure, this);
	}catch (std::exception &e) {
		std::cout<<e.what()<<std::endl;
	}
}

void SonarBelt::threadMeasure(){
	while(1){
		std::cout<<"threadMeasures "<<_thMeasure->get_id()<<" launched: state = wait\n";
		// Wait until auto-measure is true
		std::unique_lock<std::mutex> lk(_m);
		while(!_autoMeasure){
			if(_stateThreadMeas != waiting)
				_stateThreadMeas = waiting;
			_cv.wait(lk);
		}

		// Now it is locked and ready
		_stateThreadMeas = processing;
		std::cout<<"threadMeasures(): state = processing, _count = "<<_nbRevolu<<std::endl;
		// TODO! process
		std::this_thread::sleep_for (std::chrono::milliseconds(500));
		_nbRevolu++;

	    // Manual unlocking is done before notifying, to avoid waking up
	    // the waiting thread only to block again (see notify_one for details)
		lk.unlock();
		_cv.notify_one();
	}
}

void SonarBelt::playAutoMeasure(){
	if(_stateThreadMeas != notLaunched){
		std::lock_guard<std::mutex> lk(_m);
		_autoMeasure = true;
		std::cout << "_autoMeasure = true\n";
		_cv.notify_one();
	}
}

void SonarBelt::pauseAutoMeasure(){
	if(_stateThreadMeas != notLaunched){
		std::lock_guard<std::mutex> lk(_m);
		_autoMeasure = false;
		std::cout << "_autoMeasure = false\n";
		_cv.notify_one();
	}
}

int SonarBelt::getNbRevo(){
	std::unique_lock<std::mutex> lk(_m);
	int nb = _nbRevolu;
	lk.unlock();
	return nb;
}

/**
 *
 * @param id
 * @param typeInfo
 * @return
 */
int SonarBelt::readSonarInfo(eIdSonar id, eSRF02_Info typeInfo){
	int res = -1;

	// Check if i2c is not use by thread measure
	pauseAutoMeasure();
	std::unique_lock<std::mutex> lk(_m);
	while(_stateThreadMeas != waiting)
			_cv.wait(lk);

	// Get the info
	res = getInfoSonar(id, typeInfo);

	// Continue the auto-measure
	playAutoMeasure();

	return res;
}

void SonarBelt::writeSonarCmd(eIdSonar id, eSRF02_Cmd typeCmd){
	// Check if i2c is not use by thread measure
	pauseAutoMeasure();
	std::unique_lock<std::mutex> lk(_m);
	while(_stateThreadMeas != waiting)
			_cv.wait(lk);

	// Send the cmd to the sonar
	setCmdSonar(id, typeCmd);

	// Continue the auto-measure
	playAutoMeasure();
}

int SonarBelt::readSonarVers(eIdSonar id){
	return readSonarInfo(id, srf02_version);
}

int SonarBelt::readSonarDist(eIdSonar id){
	return readSonarInfo(id, srf02_range);
}

int SonarBelt::readSonarAutotuneMin(eIdSonar id){
	return readSonarInfo(id, srf02_autotuneMin);  //FIXME: not guarantee to have ms
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
		std::cout<<"Unvalid dur = "<<dur_ms.count()<<" ms\n";  // Just to evaluate the method
		dur_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
				std::chrono::high_resolution_clock::now() - startTimeRev);
	}

	for(int i=0; i<(int)_orderToProccess.size(); i++){  // for a revolution
		for(int j=0; j<(int)_orderToProccess[0].size(); j++){  // for a sonar on a revo
			eIdSonar id = _orderToProccess[i][j];

			std::unique_lock<std::mutex> lk(_m);
			_sonars.find(id)->second->updateLastDist(_sonars.find(id)->second->readSRF02_info(srf02_range));
			_sonars.find(id)->second->updateTimeIdx(_nbRevolu);
			lk.unlock();
		}
	}
}


//// For debug purpose
#ifdef DBG

#endif
