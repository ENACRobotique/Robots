/*
 * SonarBelt.cpp
 *
 *  Created on: Mar 26, 2016
 *      Author: yoyo
 */

#include "SonarBelt.h"


SonarBelt::SonarBelt(uint8_t idI2C, const listPoseSonars_t listPoseSonars) {
	_nbSonars = (int) listPoseSonars.size();
	_addrCurSonar = 0x00;

	// Initialize and open I2C
	_idI2C = idI2C;
	_fileName = std::string("/dev/i2c-");
	_fileName.append(std::to_string(_idI2C).c_str());
	if((_file = open(_fileName.c_str(), O_RDWR)) < 0){ // TODO close in destructor
#ifdef DBG
		std::cout<<_fileName<<" opening: failed\n";
#endif
		exit(1);
	}

	// Initialize the sonars
	listPoseSonars_t::const_iterator it;
	for(it = listPoseSonars.begin(); it!= listPoseSonars.end(); ++it){
		_sonars.insert(std::pair<unsigned char,SRF02>(it->first, SRF02(it->first),
				SRF02(it->second)));
	}

	// Test connections with the sonars
//#ifdef DBG
//	int addr;
//	for(int i=0; i<_nbSonars; i++){
//		addr = _sonars[i].getAddr();
//		if(ioctl(_file, I2C_SLAVE, addr) < 0){
//#ifdef DBG
//			std::cout<<"Cannot access to sonar "<<addr<<std::endl;
//#endif
//			exit(1);
//		}
//		int reg = 0x00;
//		int res;
//		if((res = i2c_smbus_read_word_data(_file, reg)) < 0){
//#ifdef DBG
//			std::cout<<"Cannot access to the file "<<_file<<" and register "<<reg<<std::endl;
//#endif
//			exit(1);
//		}
//	}
//#endif


	//
}

int SonarBelt::getSonarInfo(unsigned char idSonar, eSRF02_Info typeInfo){
	int res = -1;
	int addr = _sonars.find(idSonar)->second.getAddr();

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

	return res;
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
