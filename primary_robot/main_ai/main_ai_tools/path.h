/*
 * path.h
 *
 *  Created on: 18 févr. 2015
 *      Author: seb
 */

#ifndef AI_PATH_H_
#define AI_PATH_H_

#include "tools.h"

#include <vector>


#define MAX_RETRIES 1 //FIXME bug with simulation


using namespace std;


typedef struct {
    sPt_t p1;
    sPt_t p2;
    sObs_t obs;

    sNum_t arc_len;
    sNum_t seg_len;

    unsigned short sid;
} sTrajEl_t;

typedef struct {
    sNum_t dist;
    unsigned short tid;

    unsigned int path_len;
    sTrajEl_t *path;
} sPath_t;


class Path {
    public:
        Path();
        Path(vector <sTrajEl_t*> list); //FIXME
        ~Path();
        void clear();

        //Send methods
        void sendRobot();
        void stopRobot();
        void go2Point(const sPt_t &robot, const sPt_t &dest, const bool f); //TODO "f" to force the robot to go, even if the destination point is in obstacle.
        void followPath(vector <sObs_t> &_obs, vector <iABObs_t> &l);

        //Get methods
        void const print(); //TODO Idee : reutiliser le parseur yaml
        void const printElTraj(const unsigned int num);

        //Set methods
        sNum_t length();
        void addPath(vector <sTrajEl_t*> list);
        void addPath2(sPath_t &path);


    private:
        void setPathLength();

        sNum_t _dist;               //Computed sends
        unsigned int _path_len;     //Computed sends
        vector <sTrajEl_t*> _path;
};


void followProg();

#endif /* AI_PATH_H_ */

