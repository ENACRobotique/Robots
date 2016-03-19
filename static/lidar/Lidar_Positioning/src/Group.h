/**
 * Project Untitled
 */


#ifndef _GROUP_H
#define _GROUP_H

#include <vector>
#include <ostream>

using namespace std;

class Group {
public: 
    
    /**
     * @param dist
     * @param azimut
     */
    int appendPoint(int dist, int azimut);
    
    int computeParameters();
    
    int getAzimut();
    
    int getDist();
    
    int getMinSize();
    
    int getMaxSize();
    
    /**
     * @param Group other
     */
    int getDistOtherGroup(Group other);
    
    int getNbPoints();
    
    /**
     * @param ostream &flux
     */
    void afficher(ostream &flux);
private: 
    int azimut;
    int sizeMin;
    int sizeMax;
    int distance;
    vector<*PtLidar> points;
    void nbPoints;
};

#endif //_GROUP_H
