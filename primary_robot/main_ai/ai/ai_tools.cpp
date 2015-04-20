/*
 * ai_tools.cpp
 *
 *  Created on: 29 mars 2014
 *      Author: Sebastien Malissard
 */

#include <iomanip>

#include "ai_tools.h"
#include "obj_tools.h"
#include "tools.h"

//#define AI_TOOLS

/*
 * Print to the screen the list of active obstacle
 */
void printObsActive(vector<astar::sObs_t>& obs) {
    logs << INFO << "List of obs[i].active :\n";
    for (unsigned int i = 0; i < obs.size(); i++)
        logs << "    obs[" << i << "].active=" << obs[i].active << "\n";
}


/*
 * Returns the first obstacle number find if this point is inside else 0.
 */
unsigned int checkPointInObs(const Point2D<float>& p, vector<astar::sObs_t>& obs) {
    for (unsigned int i = 1; i < obs.size() - 1; i++) {
        if (obs[i].active == 0)
            continue;
        Point2D<float> c(obs[i].c.x, obs[i].c.y);
        if (c.distanceTo(p) < obs[i].r){
#ifdef AI_TOOLS
            logs << DEBUG << fixed << setprecision(2) << "Point is inside obs n°" << i << " (" << obs[i].c.x << " ; " << obs[i].c.y << ")";
#endif
            return i;
        }
    }
    return 0;
}

/*
 * Project the point if inside an obstacle
 */
Point2D<float> projectPointInObs(const Point2D<float>& p, vector<astar::sObs_t>& obs){
    unsigned int n = checkPointInObs(p, obs);

    if ( n > 0) {
        Circle2D<float> c(obs[n].c.x, obs[n].c.y, obs[n].r);
        Point2D<float> r;

        r = c.project(p);
        r.x += SIGN(r.x - c.c.x)*0.1;
        r.y += SIGN(r.y - c.c.y)*0.1;

#ifdef AI_TOOLS
            logs << DEBUG << fixed << setprecision(2) << "Old position : (" << p.x << " ; " << p.y << ")";
            logs << DEBUG << fixed << setprecision(2) << "New position : (" << r.x << " ; " << r.y << ")";
#endif

        //TODO if the point is in several obstacle !
        //TODO MAX of projection

        return r;
    }

    return p;
}

/*
 * Stops the robot if collision detected
 */
void colissionDetection(){
    Point2D<float> ptPr = statuses.getLastPosXY(ELT_PRIMARY);
    float anglePr = statuses.getLastOrient(ELT_PRIMARY);
    Point2D<float> ptAPr = statuses.getLastPosXY(ELT_ADV_PRIMARY);
    Point2D<float> ptASc = statuses.getLastPosXY(ELT_ADV_SECONDARY);
    sNum_t d, dot;
    int contact = 0;

    d = ptPr.distanceTo(ptAPr);
    Vector2D<float> v1(cos(anglePr), sin(anglePr)), v2(ptPr, ptAPr);
    dot = v1*v2;

    if (d < 50 && dot > 0.6 * d) {
        logs << INFO << "CONTACT PRIM ADV!!!!!!!!!!!!!!!!!!!!!!!!!";
        contact = 1;
    }

    d = ptPr.distanceTo(ptASc);
    Vector2D<float> v3(ptPr, ptASc);
    dot = v1 * v3;

    if (d < 40 && dot > 0.6 * d) {
        logs << INFO << "CONTACT SEC ADV!!!!!!!!!!!!!!!!!!!!!!!!!";
        contact = 1;
    }

    if (contact) {
        path.stopRobot(true);
    }
}

/*
 * FIXME Check the utility
 */
void posPrimary(vector<astar::sObs_t>& obs) { //FIXME permet de deplacer les objects mobile en cas de contact
    int i/*, N = obs.size()*/;
    Point2D<float> pt;
    Point2D<float> _current_pos = statuses.getLastPosXY(ELT_PRIMARY);

    if (((i = checkPointInObs(_current_pos, obs)) != 0)) {
        if (obs[i].moved == 1) {
            pt = {obs[0].c.x, obs[0].c.y};
            Circle2D<float> cir(obs[i].c.x, obs[i].c.y, obs[i].r);
            pt = cir.project(pt);
            Vector2D<float> v(pt, {obs[0].c.x, obs[0].c.y});

            obs[i].c.x += v.x;
            obs[i].c.y += v.y;

     //       obs_updated[i]++;
        }
        Point2D<float> p(_current_pos.x, _current_pos.y);
        Circle2D<float> c(obs[i].c.x, obs[i].c.y, obs[i].r);
        p = c.project(p);
        _current_pos = {p.x, p.y};
        if (sqrt(pow(_current_pos.x - obs[0].c.x, 2) + pow(_current_pos.y - obs[0].c.y, 2) < 2)) {

            obs[0].c = {_current_pos.x, _current_pos.y};
    //        obs_updated[0]++;
        }
        else {
            _current_pos = {obs[0].c.x, obs[0].c.y};
     //       obs_updated[0]++;
        }
    }
    //if non holmic
    float theta_robot = statuses.getLastOrient(ELT_ADV_PRIMARY);
    Point2D<float> p = {obs[0].c.x, obs[0].c.y};
    updateNoHaftTurn(theta_robot * 180 / M_PI, pt, obs);
//    obs_updated[N - 5]++;
 //   obs_updated[N - 6]++;
 //   obs_updated[N - 7]++;
    //end if
}


//TODO Optimisation des deplacements du robot algorithme arbre recouvrant


