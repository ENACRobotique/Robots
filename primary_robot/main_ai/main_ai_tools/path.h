/*
 * path.h
 *
 *  Created on: 18 févr. 2015
 *      Author: seb
 */

#ifndef AI_PATH_H_
#define AI_PATH_H_

#include <vector>
#include <deque>

#include "GeometryTools.h"
#include "messages-locomotion.h"

#include <a_star_tools.h>
#include "a_star.h"

#define MAX_RETRIES 5


typedef struct{
    uint32_t t1;   // in us (synchronized time):
    uint32_t t2;

    Point2D<float> p1;
    Point2D<float> p2;

    astar::sObs_t obs;

    float theta1;
    float theta2;

    float arc_len;
    float seg_len;

    bool rot1_dir; // (false: CW | true: CCW)
    bool rot2_dir;
}sTrajOrientEl_t;

template <typename T>
class TrajPosSpdEl {
public:
    uint32_t t;    // in us (synchronized time):

    Point2D<T> p;
    T v;            // unsigned

    T theta;
    T v_theta;      // signed

    T len;

    eTrajStepType type_e;

    union type {
        ~type() {};

        struct {
        } line;

        struct  {
            Circle2D<float> c;
        } arc;
    } type;
};


class Path {
    public:
        Path();
        Path(std::vector <sTrajEl_t*> list);
        Path(std::vector <sTrajOrientEl_t*> list);
        ~Path();
        void clear();
        void maintenace();

        // Send methods
        void sendRobot(bool holo, float thetaEnd);
        void stopRobot(bool holo);
        void go2PointOrient(const Point2D<float> &dest, std::vector<astar::sObs_t>& obs, float angle);
        void go2Point(const Point2D<float> &dest, const bool f, std::vector<astar::sObs_t>& obs, bool holo); //TODO "f" to force the robot to go, even if the destination point is in obstacle.
        void followPath(std::vector <astar::sObs_t>& obs, std::vector <astar::iABObs_t> &l, bool holo);
        void convPathToPathOrient(float thetaEnd);
        void computePathHolonomic(float theta_end_obj);

        // Get methods
        PointOrient2D<float> getPosOrient(uint32_t _time_us /*synchronize time*/);

        // Print
        void print() const; //TODO Idee : reutiliser le parseur yaml
        void printElTraj(const unsigned int num) const;
        void printElTrajOrient(const unsigned int num) const;
        void printPath() const;

        // Set methods
        sNum_t length();
        void addPath(std::vector <sTrajEl_t*> list);
        void addPath2(sPath_t path);

    private:
        void setPathLength();
        bool checkSameObs(astar::sObs_t& obs1, astar::sObs_t& obs2);
        bool checkSamePath(sPath_t& path);
        bool checkSamePath2(std::deque<sTrajEl_t>& path);
        bool checkSamePathOrient(std::deque<sTrajOrientEl_t>& path);
        int checkRobotBlock();
        void updateNoHaftTurn(std::vector<astar::sObs_t>& obs, std::vector<uint8_t> obs_updated); //TODO in other class

        float                               _dist;               //Computed sends
        unsigned int                        _path_len;     //Computed sends
        std::deque <sTrajEl_t>              _path;
        std::deque <sTrajOrientEl_t>        _path_orient;
        std::deque <sTrajEl_t>              _lastPathSent;
        std::deque <sTrajOrientEl_t>        _lastPathOrientSent;
        std::deque <TrajPosSpdEl<float>>    _lastTrajPosSpd;
};


void followProg();

#endif /* AI_PATH_H_ */

