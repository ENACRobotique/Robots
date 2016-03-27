/*
 * SonarBelt.cpp
 *
 *  Created on: Mar 26, 2016
 *      Author: yoyo
 */

#include "SonarBelt.h"


SonarBelt::SonarBelt(uint8_t idI2C, const listSonar_t initListSonars) {
	_nbSonars = (int) initListSonars.size();
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
	_listSonars = initListSonars;

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

int SonarBelt::readSonarInfo(eIdSonar id, eSRF02_Info typeInfo){
	int res = -1;
	int addr = _listSonars.find(id)->first;

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
