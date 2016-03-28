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
#include <thread>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <exception>

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
using listAddrPoseSonars_t = std::pair<uint8_t, PoseSonar_t>;
using listSonar_t = std::map<eIdSonar, listAddrPoseSonars_t>;

// Eurobot 2016
const listSonar_t initListSonars1 = {
	 // {id, {addr, {alpha, r}}}
		{s1, {0xE0, {O_S, 100}}},
		{s2, {0xE2, {O_S+D_S, 100}}},
		{s3, {0xE4, {O_S+D_S*2, 100}}},
		{s4, {0xE6, {O_S+D_S*3, 100}}},
		{s5, {0xE8, {O_S+D_S*4, 100}}},
		{s6, {0xEA, {O_S+D_S*5, 100}}},
        {s7, {0xEC, {O_S+D_S*6, 100}}},
        {s8, {0xEE, {O_S+D_S*7, 100}}},
        {s9, {0xF0, {O_S+D_S*8, 100}}},
	    {s10, {0xF2, {O_S+D_S*9, 100}}},
	    {s11, {0xF4, {O_S+D_S*10, 100}}},
	    {s12, {0xF6, {O_S+D_S*11, 100}}}
};

typedef enum eStateThread{
	notLaunched,
	processing,
	waiting,
	eStateThread_Max
}eStateThread;

class SonarBelt {
public:
	SonarBelt(int idI2C, const listSonar_t listPoseSonars, bool openI2C = false,
			int file = 0);
	int readSonarInfo(eIdSonar id, eSRF02_Info typeInfo);
	void writeSonarCmd(eIdSonar id, eSRF02_Cmd typeCmd);
	int readSonarVers(eIdSonar id);
	int readSonarDist(eIdSonar id);
	int readSonarAutotuneMin(eIdSonar id);
	void launchBurst(eIdSonar id);
	int getNbRevo();
	bool getAutoMeasure() const {return _autoMeasure;};
	// Thread
	void playAutoMeasure();
	void pauseAutoMeasure();


private:
	uint8_t _nbSonars;
	int _idI2C;
	uint8_t _addrCurSonar;
	std::string _fileName;
	int _file;
	listSonar_t _listSonars;
	std::vector<float> _lastDistances;  // Concurrent access
	int _nbRevolu;  // Concurrent access
	bool _autoMeasure;
	std::mutex _m;
	std::condition_variable _cv;
	std::thread* _thMeasure;
	eStateThread _stateThreadMeas;


	int readRegister(int reg);
	void writeRegister(int reg, int val);
	bool startComWithSonar(uint8_t idSonar);
	void threadMeasure();
	void startThreadMeasure();
};

#endif /* SONARBELT_H_ */
