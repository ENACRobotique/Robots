/**
 * Project Untitled
 */


#ifndef _ACQLIDAR_H
#define _ACQLIDAR_H

#include <vector>
#include "PtLidar.h"
#include "serialib.h"

#define NB_PTS 360

using namespace std;

class AcqLidar {
public: 
    AcqLidar();
    int updateData();
    
    vector<PtLidar> getData();
    
    int init();
    void close();
private: 
    void updateBuff(int angle, unsigned char * data);
    int calc_checksum(unsigned char * data);
    PtLidar buffData[NB_PTS];
    int nbRevo;
    serialib serial;
};

#endif //_ACQLIDAR_H
