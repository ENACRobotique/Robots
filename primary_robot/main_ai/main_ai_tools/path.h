/*
 * path.h
 *
 *  Created on: 18 févr. 2015
 *      Author: seb
 */

#ifndef AI_PATH_H_
#define AI_PATH_H_

#include <a_star_tools.h>
#include <vector>
#include <deque>

#define MAX_RETRIES 1 //FIXME bug with simulation


using namespace std;

typedef struct{
    float t1;   // in us (synchronized time):
    float t2;

    sPt_t p1;
    sPt_t p2;

    sObs_t obs;//TODO change in circle type with the new math tools

    float theta1;
    float theta2;

    float arc_len;
    float seg_len;

    bool rot1_dir; // (false: CW | true: CCW)
    bool rot2_dir;
}sTrajOrientEl_t;

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
        Path(vector <sTrajEl_t*> list);
        Path(vector <sTrajOrientEl_t*> list);
        ~Path();
        void clear();
        void maintenace();

        //Send methods
        void sendRobot();
        void stopRobot();
        void go2Point(const sPt_t &dest, const bool f); //TODO "f" to force the robot to go, even if the destination point is in obstacle.
        void followPath(vector <sObs_t> &_obs, vector <iABObs_t> &l);
        void convPathToPathOrient();
        void computeOrientPathForHolonomic(float theta_end_obj);
        void computeTimePathForHolonomic();

        //Get methods
        void print() const; //TODO Idee : reutiliser le parseur yaml
        void printElTraj(const unsigned int num) const;
        void printElTrajOrient(const unsigned int num) const;
        void printPath() const;

        //Set methods
        sNum_t length();
        void addPath(vector <sTrajEl_t*> list);
        void addPath2(sPath_t path);


    private:
        void setPathLength();
        bool checkSameObs(sObs_t& obs1, sObs_t& obs2);
        bool checkSamePath(sPath_t& path);
        bool checkSamePath2(deque<sTrajEl_t>& path);
        int checkRobotBlock();
        void updateNoHaftTurn() ;

        sNum_t _dist;               //Computed sends
        unsigned int _path_len;     //Computed sends
        deque <sTrajEl_t> _path;
        deque <sTrajOrientEl_t> _path_orient;
};


void followProg();

#endif /* AI_PATH_H_ */

