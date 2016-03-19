/**
 * Project Untitled
 */


#ifndef _PTLIDAR_H
#define _PTLIDAR_H

typedef struct PtLidar {
    int azimut;
    int distance;
    int quality;
    bool valid;
    bool warning;
    int updTour;
} PtLidar;

/*
class PtLidar {
public: 
    
    int getAzimut();
    
    int getDist();
private: 
    int azimut;
    int distance;
    int quality;
    bool valid;
    int updTour;
};*/

#endif //_PTLIDAR_H
