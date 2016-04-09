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

#define O_S SRF02_APERT_ANGLE/2
#define D_S 30

typedef enum eIdSonar{
	s1, s2, s3,
	s4, s5, s6,
	s7, s8, s9,
	s10, s11, s12,
	eIdSonar_Max
}eIdSonar;
using listAddrPoseSonars_t = std::pair<uint8_t, PoseSonar_t>;
using listSonar_t = std::map<eIdSonar, listAddrPoseSonars_t>;
using mapSonars_t = std::map<eIdSonar, SRF02*>;
using orderToProcess_t = std::vector<std::vector<eIdSonar>>;
using sonarsDist_t = std::map<eIdSonar, int>;

typedef enum eStateThread{
	notLaunched,
	processing,
	waiting,
	eStateThread_Max
}eStateThread;

// _________________ Eurobot 2016 _________________________
const listSonar_t initListSonars1 = {
	 // {id, {addr, {alpha, r}}}  // template
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

const orderToProcess_t orderToProcess_3PerRev = {
		{s1, s5, s9},  // First revo
		{s2, s6, s10},  // 2nd revo
		{s3, s7, s11},
		{s4, s8,s12}
};
const orderToProcess_t orderToProcess_4PerRev = {
		{s1, s4, s7, s10},  // First rev
		{s2, s5, s8, s11},
		{s3, s6, s9, s12}  // 3rd revo
};
// ________________________________________________________

class SonarBelt {
public:
	SonarBelt(int idI2C, const listSonar_t listPoseSonars, const orderToProcess_t order);
	int readSonarInfo(eIdSonar id, eSRF02_Info typeInfo);
	void writeSonarCmd(eIdSonar id, eSRF02_Cmd typeCmd);
	int readSonarVers(eIdSonar id);
	int readSonarDist(eIdSonar id);
	int readSonarAutotuneMin(eIdSonar id);
	void launchBurst(eIdSonar id);
	int getNbRevo();
	bool getAutoMeasure() const {return _autoMeasure;};
	void doMeasure_revol();
	// Thread
	void playAutoMeasure();
	void pauseAutoMeasure();


private:
	uint8_t _nbSonars;
	int _nbRevolu;  // Concurrent access
	mapSonars_t _sonars;
	orderToProcess_t _orderToProccess;

	// thread for auto_measure
	bool _autoMeasure;
	std::mutex _m;
	std::condition_variable _cv;
	std::thread* _thMeasure;
	eStateThread _stateThreadMeas;

	int getInfoSonar(eIdSonar id, eSRF02_Info infoType);
	void setCmdSonar(eIdSonar id, eSRF02_Cmd cmdType);
	void threadMeasure();

	//// For debug purpose
#ifdef DBG
public:
#endif
};

#endif /* SONARBELT_H_ */
