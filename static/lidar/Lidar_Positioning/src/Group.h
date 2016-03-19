/**
 * Class header file for Group
 */


#ifndef _GROUP_H
#define _GROUP_H

#include <vector>
#include <ostream>
#include "PtLidar.h"
#include "math.h"

using namespace std;

#define DEG_TO_RAD (M_PI/180.)

class Group {
public: 
    
    /**
     * @param dist
     * @param azimut
     */
    void appendPoint(PtLidar * pt);
    
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
    void afficher(ostream &flux) const;

    friend ostream &operator<<( ostream &flux, const Group& grp);
private: 
    int azimut;
    int sizeMin;
    int sizeMax;
    int distance;
    vector<PtLidar *> points;
    int nbPoints;
};

#endif //_GROUP_H
