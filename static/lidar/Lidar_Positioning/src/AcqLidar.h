/**
 * Project Untitled
 */


#ifndef _ACQLIDAR_H
#define _ACQLIDAR_H

#include <vector>

#define NB_PTS 360

using namespace std;

class AcqLidar {
public: 
    
    int updateData();
    
    vector<PtLidar> getData();
    
    int init();
private: 
    PtLidar buffData[NB_PTS];
    int nbRevo;
    serialib serial;
};

#endif //_ACQLIDAR_H
