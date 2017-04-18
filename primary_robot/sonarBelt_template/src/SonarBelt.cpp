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
	std::cout<<"InitListSonar has "<<initListSonars.size()<<" sonars."<<std::endl;

	_nbRevolu = 0;
	_orderToProccess = order;

	// Create the sonars
#ifdef DBG_SONAR_BELT
	std::cout<<"Initialize list of SRF02: starting\n";
#endif
	for(listSonar_t::const_iterator it=initListSonars.begin();
			it!= initListSonars.end(); ++it){
#ifdef DBG_SONAR_BELT
		std::cout<<"SRF02 start initialization: id = "<<it->first<<", address = "<<it->second.first<<
				", orient = "<<it->second.second.first<<", r = "<<it->second.second.second<<std::endl;
#endif
		_sonars[it->first] = new SRF02(it->second.first, it->second.second);
#ifdef DBG_SONAR_BELT
		std::cout<<"SRF02 initialized: id = "<<it->first<<", address = "<<it->second.first<<std::endl;
#endif
	}
	std::cout<<"Initialize list of SRF02: finished and succeed\n";

	// Test connections with the sonars
#if defined(DBG_SONAR_BELT)  &&  defined(DBG_SRF02)
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
	double dist;

	_m_data.lock();
	dist = _sonars.find(id)->second->getLastDist();
	_m_data.unlock();

	return dist;
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
//	std::chrono::duration<float> dur_s;
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
			lk.unlock();
		}
	}

	std::unique_lock<std::mutex> lk(_m_data);
	_nbRevolu++;
	lk.unlock();
}

/**
 *
 * @param theta an angle (degree) in the basis of the robot wrt the y axis (cap) of the robot.
 *        This angle represents the orientation of the velocity vector of the robot.
 * @param delta is the half width of the desired cone of detection (degree)
 * @return a vector of pair of doubles, in which each pair (angle, distance) is
 *        linked to the range measured by a sonar.
 */
std::vector<std::pair<double, double>> SonarBelt::scanInThisDirect(const double theta, const double delta)const{
	std::vector<std::pair<double, double>> res;
	double angle = (theta<0)?
			       ((theta<360)? (angle = 0): (angle = theta + 360.)):
		           ((theta>360)? (angle = 0): (angle = theta));
	double angle_cur, angle_next;
	double aper = (delta>0)? (delta): (-delta);

	// Compute the necessary number of sonars required to cover the delta angle
	int nb_half_sonars = (int)aper/SRF02_APERT_ANGLE + 1;

	// Find the median
	mapSonars_t::const_iterator it_cur;
	mapSonars_t::const_iterator it_next;
	for(it_cur = _sonars.begin(); it_cur!=_sonars.end(); it_cur++){
		// Construct the next iterator
		it_next = it_cur; it_next++;
		if(it_next == _sonars.end())
			it_next = _sonars.begin();

		angle_cur = it_cur->second->getPose().first;
		angle_next = it_next->second->getPose().first;
		if(angle_next < angle_cur)
			angle_cur -= 360;
		if((angle_cur <= angle)  &&  (angle < angle_next)){
			// Set the result
			// Warning use the previous values of it_cur and it_next
			for(int i=0; i<nb_half_sonars; i++){
				res.push_back(std::make_pair((double)(it_cur->second->getPose().first),
						                     (double)(it_cur->second->getLastDist())));
				res.push_back(std::make_pair((double)(it_next->second->getPose().first),
								                     (double)(it_next->second->getLastDist())));
				std::cout<<"___ pair = ("<<res[i].first<<", "<<res[i].second<<")"<<std::endl;
				it_cur--;
				it_next++;
			}
			return res;
		}
	}

	std::cout<<"Error: scanInThisDirect(): cannot identify the median of the direction\n";

	return res;
}


//// For debug purpose
#ifdef DBG

#endif
