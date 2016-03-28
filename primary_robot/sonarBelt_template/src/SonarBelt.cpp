/*
 * SonarBelt.cpp
 *
 *  Created on: Mar 26, 2016
 *      Author: yoyo
 */

#include "SonarBelt.h"


SonarBelt::SonarBelt(int idI2C, const listSonar_t initListSonars, bool openI2C,
		int file) {
	_nbSonars = (int) initListSonars.size();
	_addrCurSonar = 0x00;
	_nbRevolu = 0;
	_stateThreadMeas = notLaunched;

	// Initialize and open I2C
	_idI2C = idI2C;
	if(openI2C){
		_fileName = std::string("/dev/i2c-");
		_fileName.append(std::to_string(_idI2C).c_str());
		if((_file = open(_fileName.c_str(), O_RDWR)) < 0){ // TODO close in destructor
	#ifdef DBG
			std::cout<<_fileName<<" opening: failed\n";
	#endif
			exit(1);
		}
	}else{
		_file = file;
	}

	// Initialize the sonars
	_listSonars = initListSonars;

	// Test connections with the sonars
	listSonar_t::iterator it = _listSonars.begin();
	for(int i=0; it!=_listSonars.end(); i++, ++it){
		std::cout<<"s"<<i+1<<": vers = "<<readSonarVers(it->first)<<std::endl;
	}

	// Initialize thread for permanent measures
	try{
		this->startThreadMeasure();
	}catch (std::exception &e) {
		std::cout<<e.what()<<std::endl;
	}
}

void SonarBelt::startThreadMeasure(){
	_thMeasure = new std::thread(&SonarBelt::threadMeasure, this);
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

int SonarBelt::readSonarInfo(eIdSonar id, eSRF02_Info typeInfo){
	int res = -1;
	int addr = _listSonars.find(id)->first;

	// Check if i2c is not use by thread measure
	pauseAutoMeasure();
	std::unique_lock<std::mutex> lk(_m);
	while(_stateThreadMeas != waiting)
			_cv.wait(lk);

	if(!startComWithSonar(addr))
		return res;

	switch(typeInfo){
	case srf02_version:
		res = readRegister(REG_SOFT_REV);
		break;
	case srf02_range:
		res = readRegister(REG_RANGE_H)*256;
		res += readRegister(REG_RANGE_L);
		break;
	case srf02_autotuneMin:
		res = readRegister(REG_AUTOTUNE_MIN_H)*256;
		res += readRegister(REG_AUTOTUNE_MIN_L);
		break;
	default:
#ifdef DBG
		std::cout<<"Unknown sonar type info: "<<typeInfo<<std::endl;
#endif
	}

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

	if(!startComWithSonar(_listSonars.find(id)->first))
			return;

	switch(typeCmd){
	case srf02_mes_inch:
		writeRegister(REG_CMD, CMD_MES_INCH);
		break;
	case srf02_mes_cm:
		writeRegister(REG_CMD, CMD_MES_CM);
		break;
	case srf02_mes_ms:
		writeRegister(REG_CMD, CMD_MES_MS);
		break;
	case srf02_fakeMes_inch:
		writeRegister(REG_CMD, CMD_FAKE_MES_INCH);
		break;
	case srf02_fakeMes_cm:
		writeRegister(REG_CMD, CMD_FAKE_MES_CM);
		break;
	case srf02_fakeMes_ms:
		writeRegister(REG_CMD, CMD_FAKE_MES_MS);
		break;
	case srf02_burst:
		writeRegister(REG_CMD, CMD_BURST);
		break;
	case srf02_autotuneCmd:
		writeRegister(REG_CMD, CMD_AUTOTUNE);
		break;
	case srf02_chgAddr:
		writeRegister(REG_CMD, CMD_CHG_ADD_1);  // Check the procedure
		writeRegister(REG_CMD, CMD_CHG_ADD_2);
		writeRegister(REG_CMD, CMD_CHG_ADD_3);
		break;
	default:
#ifdef DBG
		std::cout<<"Unknown sonar type cmd: "<<typeCmd<<std::endl;
#endif
	}

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

int SonarBelt::readRegister(int reg){
	int res = -1;
	if((res = i2c_smbus_read_word_data(_file, reg)) < 0){
#ifdef DBG
		std::cout<<"Cannot access to the file "<<_file<<" and register "<<reg<<std::endl;
#endif
	}

	return res;
}

void SonarBelt::writeRegister(int reg, int val){
	if(i2c_smbus_write_word_data(_file, reg, val) < 0){
#ifdef DBG
		std::cout<<"Cannot write value = "<<val<<" to the file "<<_file<<
				" and register "<<reg<<std::endl;
#endif
	}
}

bool SonarBelt::startComWithSonar(uint8_t idSonar){
	if(idSonar != _addrCurSonar){
		if(ioctl(_file, I2C_SLAVE, idSonar) < 0){
#ifdef DBG
			std::cout<<"Cannot access to sonar "<<idSonar<<std::endl;
#endif
			return false;
		}
	}

	return true;
}

void SonarBelt::launchBurst(eIdSonar id){
	writeSonarCmd(id, srf02_burst);
}
