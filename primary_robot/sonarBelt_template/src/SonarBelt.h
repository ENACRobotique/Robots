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

typedef std::pair<int, int> PoseSonar_t;
typedef std::map<unsigned char, PoseSonar_t> listPoseSonars_t;

// Eurobot 2016
const listPoseSonars_t listPoseSonar1 = {
		{0xE0, {O_S, 100}}, {0xE2, {O_S+D_S, 100}}, {0xE4, {O_S+D_S*2, 100}},  // 1st quadrant
		{0xE6, {O_S+D_S*3, 100}}, {0xE8, {O_S+D_S*4, 100}}, {0xEA, {O_S+D_S*5, 100}},  // 2nd quadrant
		{0xEC, {O_S+D_S*6, 100}}, {0xEE, {O_S+D_S*7, 100}}, {0xF0, {O_S+D_S*8, 100}},  // 3rt quadrant
		{0xF2, {O_S+D_S*9, 100}}, {0xF4, {O_S+D_S*10, 100}}, {0xF6, {O_S+D_S*11, 100}}  // 4th quadrant
};


class SonarBelt {
public:
	SonarBelt(int idI2C, const listPoseSonars_t listPoseSonars);
	int getSonarInfo(unsigned char idSonar, int typeInfo);
	int getSonarDist_cm(unsigned char idSonar);
	int getSonarDist_inch(unsigned char idSonar);
	int getSonarDist_ms(unsigned char idSonar);
	int getSonarFakeDist_cm(unsigned char idSonar);
	int getSonarFakeDist_inch(unsigned char idSonar);
	int getSonarFakeDist_ms(unsigned char idSonar);

private:
	int _nbSonars;
	int _idI2C;
	std::string _fileName;
	int _file;
	std::vector<SRF02> _sonars;
	std::vector<float> _lastDistances;
};

#endif /* SONARBELT_H_ */
