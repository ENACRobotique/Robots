/*
 * SonarBelt.cpp
 *
 *  Created on: Mar 26, 2016
 *      Author: yoyo
 */

#include "SonarBelt.h"


SonarBelt::SonarBelt(int idI2C, const listPoseSonars_t listPoseSonars) {
	_nbSonars = (int) listPoseSonars.size();

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

	// Test connections with the sonars
#ifdef DBG
	int addr;
	for(int i=0; i<_nbSonars; i++){
		addr = _sonars[i].getAddr();
		if(ioctl(_file, I2C_SLAVE, addr) < 0){
#ifdef DBG
			std::cout<<"Cannot access to sonar "<<addr<<std::endl;
#endif
			exit(1);
		}
		int reg = 0x00;
		int res;
		if((res = i2c_smbus_read_word_data(_file, reg)) < 0){
#ifdef DBG
			std::cout<<"Cannot access to the file "<<_file<<" and register "<<reg<<std::endl;
#endif
			exit(1);
		}
	}
#endif

	// Initialize the sonars
	listPoseSonars_t::const_iterator it;
	for(it=listPoseSonars.begin(); it!=listPoseSonars.end(); ++it){
		_sonars.push_back(SRF02(it->first));
	}

	//
}

