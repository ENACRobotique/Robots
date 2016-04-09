/*
 * SRF02.cpp
 *
 *  Created on: Mar 20, 2016
 *      Author: yoyo
 */

#include "SRF02.h"

/**
 * @brief SRF02 CTor, initializing SRF02 with address and mode
 * @param ard I2C Device address in hexadecimal
 * @param mode ranging mode
 */
SRF02::SRF02(uint8_t addr, PoseSonar_t p) {
	_addr = addr;  // Can be used like an id
	_startTime = 0;
	_pose = p;
	_unit = cm;
	_fd = 0;

	// Set the file descriptor
	_fd = wiringPiI2CSetup(_addr);

#ifdef DBG_SRF02
	printf("Created SRF02 sonar: addr = %d, pose = (%d, %d)\n",
			addr, _pose.first, _pose.second);
#endif
}

uint8_t SRF02::getAddr(){
	return _addr;
}

PoseSonar_t SRF02::getPose(){
	return _pose;
}

eSRF02_unit SRF02::getUnit(){
	return _unit;
}

int SRF02::getMeasDelay_ms(){
	return SRF02_MEAS_PERIOD;
}

int SRF02::readSRF02_info(eSRF02_Info typeInfo){
	int res = -1;

	switch(typeInfo){
	case srf02_version:
		res = wiringPiI2CReadReg8(_fd, REG_SOFT_REV);
		break;
	case srf02_range:
		res = wiringPiI2CReadReg8(_fd, REG_RANGE_H)*256;
		res += wiringPiI2CReadReg8(_fd, REG_RANGE_L);
		break;
	case srf02_autotuneMin:
		res = wiringPiI2CReadReg8(_fd, REG_AUTOTUNE_MIN_H)*256;
		res += wiringPiI2CReadReg8(_fd, REG_AUTOTUNE_MIN_L);
		break;
#ifdef DBG
	default:
		std::cout<<"Unknown sonar type info: "<<typeInfo<<std::endl;
#endif
	}

	return res;
}

bool SRF02::writeSRF02_cmd(eSRF02_Cmd typeCmd){
	switch(typeCmd){
	case srf02_mes_inch:
		wiringPiI2CWriteReg8(_fd, REG_CMD, CMD_MES_INCH);
		break;
	case srf02_mes_cm:
		wiringPiI2CWriteReg8(_fd, REG_CMD, CMD_MES_CM);
		break;
	case srf02_mes_ms:
		wiringPiI2CWriteReg8(_fd, REG_CMD, CMD_MES_MS);
		break;
	case srf02_fakeMes_inch:
		wiringPiI2CWriteReg8(_fd, REG_CMD, CMD_FAKE_MES_INCH);
		break;
	case srf02_fakeMes_cm:
		wiringPiI2CWriteReg8(_fd, REG_CMD, CMD_FAKE_MES_CM);
		break;
	case srf02_fakeMes_ms:
		wiringPiI2CWriteReg8(_fd, REG_CMD, CMD_FAKE_MES_MS);
		break;
	case srf02_burst:
		wiringPiI2CWriteReg8(_fd, REG_CMD, CMD_BURST);
		break;
	case srf02_autotuneCmd:
		wiringPiI2CWriteReg8(_fd, REG_CMD, CMD_AUTOTUNE);
		break;
	case srf02_chgAddr:
		wiringPiI2CWriteReg8(_fd, REG_CMD, CMD_CHG_ADD_1);  // Check the procedure
		wiringPiI2CWriteReg8(_fd, REG_CMD, CMD_CHG_ADD_2);
		wiringPiI2CWriteReg8(_fd, REG_CMD, CMD_CHG_ADD_3);
		break;
	default:
#ifdef DBG
		std::cout<<"Unknown sonar type cmd: "<<typeCmd<<std::endl;
#endif
		return false;
	}

	return true;
}

int SRF02::get_fd(){
	return _fd;
}
