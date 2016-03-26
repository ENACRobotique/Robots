/*
 * SonarBelt.h
 *
 *  Created on: Mar 26, 2016
 *      Author: yoyo
 */

#ifndef SONARBELT_H_
#define SONARBELT_H_

#include "SRF02.h"
#include <vector>
#include <PointOrient2D.h>
#include <map>
#include <linux/i2c-dev.h>
#include <string>
#include <iostream>
#include <fcntl.h>
#include <sys/ioctl.h>

#define DBG

#ifdef DBG
#include <stdio.h>
#endif

#define O_S APERTURE_ANGLE/2
#define D_S 30

typedef enum eIdSonar{
	s1, s2, s3,
	s4, s5, s6,
	s7, s8, s9,
	s10, s11, s12
}eIdSonar;
using listPoseSonars_t = std::map<uint8_t, PoseSonar_t>;
using mapIdToSonar_t = std::map<eIdSonar, uint8_t>;

// Eurobot 2016
const listPoseSonars_t listPoseSonar1 = {
		{0xE0, {O_S, 100}}, {0xE2, {O_S+D_S, 100}}, {0xE4, {O_S+D_S*2, 100}},  // 1st quadrant
		{0xE6, {O_S+D_S*3, 100}}, {0xE8, {O_S+D_S*4, 100}}, {0xEA, {O_S+D_S*5, 100}},  // 2nd quadrant
		{0xEC, {O_S+D_S*6, 100}}, {0xEE, {O_S+D_S*7, 100}}, {0xF0, {O_S+D_S*8, 100}},  // 3rt quadrant
		{0xF2, {O_S+D_S*9, 100}}, {0xF4, {O_S+D_S*10, 100}}, {0xF6, {O_S+D_S*11, 100}}  // 4th quadrant
};
const mapIdToSonar_t listMapIdSonar1 = {
		{s1, 0xE0}, {s2, 0xE2}, {s3, 0xE4},
		{s4, 0xE6}, {s5, 0xE8}, {s6, 0xEA},
		{s7, 0xEC}, {s8, 0xEE}, {s9, 0xF0},
		{s10, 0xF2}, {s11, 0xF4}, {s12, 0xF6}
};


class SonarBelt {
public:
	SonarBelt(uint8_t idI2C, const listPoseSonars_t listPoseSonars);
	int getSonarInfo(uint8_t idSonar, eSRF02_Info typeInfo);
	int getSonarInfo(eIdSonar id, eSRF02_Info typeInfo);
	int getSonarDist_cm(uint8_t idSonar);
	int getSonarDist_inch(uint8_t idSonar);
	int getSonarDist_ms(uint8_t idSonar);
	int getSonarFakeDist_cm(uint8_t idSonar);
	int getSonarFakeDist_inch(uint8_t idSonar);
	int getSonarFakeDist_ms(uint8_t idSonar);

private:
	uint8_t _nbSonars;
	uint8_t _idI2C;
	uint8_t _addrCurSonar;
	std::string _fileName;
	int _file;
	std::map<uint8_t,SRF02> _sonars;
	std::vector<float> _lastDistances;

	int readRegister(int reg);
	bool startComWithSonar(uint8_t idSonar);
};

#endif /* SONARBELT_H_ */
