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

enum class Axis{
        NONE,
        X,
        Y,
        Z
};


/*
 * Print to the screen the list of active obstacle
 */
void printObsActive(vector<astar::sObs_t>& obs) {
    logs << INFO << "List of obs[i].active :\n";
    for (unsigned int i = 0; i < obs.size(); i++)
        if(obs[i].active)
            logs << "    obs[" << i << "].active=" << "1" << "\n";
        else
            logs << "    obs[" << i << "].active=" << "0" << "\n";
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

unsigned int checkPointInLimitPlayground(const Point2D<float>& p, const float limit){

    //playground
    if(p.x < limit || p.x > 300 - limit || p.y < limit || p.y > 200 - limit)
        return 1;

    //stairs
    if(p.y > 200 - 58 - limit && p.x > 96 - limit && p.x < 204 + limit)
        return 2;

    return 0;
}

/*
 * Project the point if inside an obstacle
 */
Point2D<float> projectPointInObs(const Point2D<float>& p, vector<astar::sObs_t>& obs){
    unsigned int n = checkPointInObs(p, obs);

    if (n > 0) {
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

Point2D<float> projectPointInLimitPlayground(const Point2D<float>& p,  const float limit){

    if (checkPointInLimitPlayground(p, limit) > 0) {
        Point2D<float> r(p);
logs << INFO << "Projection detect";
        if(r.x < limit){
            Line2D<float> l({limit, 0}, (Point2D<float>){limit, 200});
            r = l.project(r);
        }
        if(r.x > 300 - limit){
            Line2D<float> l({300 - limit, 0}, (Point2D<float>){300 - limit, 200});
            r = l.project(r);
        }
        if(r.y < limit){
            Line2D<float> l({0, limit}, (Point2D<float>){300, limit});
            r = l.project(r);
        }
        if(r.y > 200 - limit){
            Line2D<float> l({0, 200 - limit}, (Point2D<float>){300, 200 - limit});
            r = l.project(r);
        }

        if(checkPointInLimitPlayground(r, limit) > 1){
            logs << INFO << "Projection detect escalier";
            if(r.y > 200 - 58 - limit){
                Line2D<float> l({0, 200 - 58 - limit}, (Point2D<float>){300, 200 - 58 - limit});
                r = l.project(r);
            }
            if(r.x > 150 && r.x < 204 + limit){
                Line2D<float> l({204 + limit, 0}, (Point2D<float>){204 + limit, 200});
                r = l.project(r);
            }
            if(r.x < 150 && r.x > 96 - limit){
                Line2D<float> l({96 - limit, 0}, (Point2D<float>){96 - limit, 200});
                r = l.project(r);
            }
        }

        return r;
    }

    return p;
}

/*
 * Return > 0 if collision detected
 */

int colissionDetection(const eElement& robot, const std::vector<astar::sObs_t>& pos){
    Point2D<float> ptPr(pos[0].c.x, pos[0].c.y);
    Point2D<float> ptSc(pos[1].c.x, pos[1].c.y);
    Point2D<float> ptAPr(pos[2].c.x, pos[2].c.y);
    Point2D<float> ptASc(pos[3].c.x, pos[3].c.y);
    Point2D<float> ptRobot(pos[robot].c.x, pos[robot].c.y);
    float d;

    d = ptRobot.distanceTo(ptPr);
    if (d < (pos[robot].r + pos[0].r) && d) {
     //   logs << INFO << "CONTACT PRIM!!!!!!!!!!!!!!!!!!!!!!!!!";
        return 1;
    }

    d = ptRobot.distanceTo(ptSc);
    if (d < (pos[robot].r + pos[1].r) && d) {
      //  logs << INFO << "CONTACT SEC!!!!!!!!!!!!!!!!!!!!!!!!!";
        return 2;
    }

    d = ptRobot.distanceTo(ptAPr);
    if (d < (pos[robot].r + pos[2].r) && d) {
     //   logs << INFO << "CONTACT PRIM ADV!!!!!!!!!!!!!!!!!!!!!!!!!";
        return 3;
    }

    d = ptRobot.distanceTo(ptASc);
    if (d < (pos[robot].r + pos[3].r) && d) {
     //   logs << INFO << "CONTACT SEC ADV!!!!!!!!!!!!!!!!!!!!!!!!!";
        return 4;
    }

    return 0;
}

Axis searchAxis(const Segment2D<float>& seg){
    if(seg.p1.x == seg.p2.x)
        return Axis::Y;
    else if(seg.p1.y == seg.p2.y)
        return Axis::X;

    return Axis::NONE;
}

void setStartingPosition(std::vector<SimpleTraj>& traj,const Point2D<float>& curPt, const float& curAngle, const Point2D<float>& destPt, const float& destAngle, const vector<Segment2D<float>>& robot, const vector<Segment2D<float>>& playground){
    float dx = -1, dy = -1, ox = 10, oy = 10;
    int pgx = -1, pgy;


    float dist_robot = 11.262;
//    float sdr = R_ROBOT; //security distance robot
    float theta_robot[2] = {
            191.63*M_PI/180.,
            191.63*M_PI/180. + 2*M_PI/3
    };
/*
    //TODO Find the best pair for the resetting the position
    std::vector<std::pair<unsigned int, unsigned int>> c;
    //x:0 , y:1
    c.push_back({0,1});


    float t  = 0;      //time
    float ls = 10;      //linear speed in cm/s
    float as = ls/14;   //angular speed in rad/s

    for(unsigned int i = 0 ; i < c.size() ; i ++){
        float tt = 0; //temporary time
        Point2D<float> dest = playground[c[i].first].project(curPt);
        Vector2D<float> vref(1,0);
        Vector2D<float> vseg(playground[c[i].first].p1, playground[c[i].first].p2);
        float oseg = fmod(vref.angle(vseg), M_PI);

        dest.x += SIGN(curPt.x - dest.x)*sdr*sin(1);
        dest.y += SIGN(curPt.y - dest.y)*sdr*cos(1);
        tt += curPt.distanceTo(dest)*ls;
    }



*/



    /******/
    traj.push_back({curPt, curAngle, {0,0}, 0});

    for(unsigned int i = 0 ; i < playground.size() ; i++){
        if(searchAxis(playground[i]) == Axis::Y){
            float d = playground[i].distance(curPt);
            if(d < dx || dx == -1){
                dx = d;
                pgx = i;
            }
        }
    }

    if(pgx != -1){
        for(unsigned int i = 0 ; i < 2 ; i++){
           /* Point2D<float> pjt = robot[i].project({0, 0});
            if(pjt != robot[i].p1 && pjt != robot[i].p2){
                Vector2D<float> v1({0,0}, pjt);
                Vector2D<float> v2(1, 0);
                float o = v2.angle(v1) + curAngle;
                if(o < ox || ox == 10 ){
                    ox = o - curAngle;
                    rx = i;
                }
            }*/
            float o = fmod(fabs(curAngle + (2*M_PI-theta_robot[i])), 2*M_PI);
            if(o < ox){
                ox = 2*M_PI-theta_robot[i];
                cout << "x=" << i << "o=" << ox*180/M_PI;
            }

        }
    }

    logs << WAR << pgx;
    if(pgx == -1)
        logs << INFO << "Can't set position in X axis";
    else{
        Point2D<float> dest = playground[pgx].project(curPt);
        dest.x += SIGN(curPt.x - dest.x)*R_ROBOT;
        float angle = ox;
        traj.push_back({dest, angle, {0,0}, 5});

        dest = playground[pgx].project(curPt);
        dest.x += SIGN(curPt.x - dest.x)*dist_robot;
        //dest.x += SIGN(curPt.x - dest.x)*robot[0].distance({0,0});
        traj.push_back({dest, angle, {dest.x, 0.}, 1});

        dest = playground[pgx].project(curPt);
        dest.x += SIGN(curPt.x - dest.x)*R_ROBOT;
        traj.push_back({dest, angle, {0,0}, 0});
    }

    for(unsigned int i = 0 ; i < playground.size() ; i++){
        if(searchAxis(playground[i]) == Axis::X){
            float d = playground[i].distance(traj.back().dest);
            if(d < dy || dy == -1){
                dy = d;
                pgy = i;
            }
        }
    }

    for(unsigned int i = 0 ; i < robot.size() ; i++){
        /*
        Point2D<float> pjt = robot[i].project({0, 0});
        if(pjt != robot[i].p1 && pjt != robot[i].p2){
            Vector2D<float> v1({0, 0}, pjt), v2(1, 0);
            float o = v2.angle(v1) + traj.back().angle;
            if(o < oy || oy == 10 ){
                oy = o - traj.back().angle;
                ry = i;
            }
        }
        */

        float o = fmod(fabs(curAngle + (2*M_PI-theta_robot[i]-M_PI_2)), 2*M_PI);
        if(o < oy){
            oy = 2*M_PI-theta_robot[i]-M_PI_2;
            cout << "x=" << i << "o=" << ox*180/M_PI;
        }
    }

    if(pgy == -1)
        logs << INFO << "Can't set position in Y axis";
    else{
        Point2D<float> lastPos = traj.back().dest;
        Point2D<float> dest = playground[pgy].project(traj.back().dest);
        dest.y += SIGN(traj.back().dest.y - dest.y)*R_ROBOT;
        //float angle = oy + SIGN(traj.back().dest.y - dest.y)*M_PI_2;
        float angle = oy;
        traj.push_back({dest, angle, {0,0}, 5});

        dest = playground[pgy].project(lastPos);
        dest.y += SIGN(lastPos.y - dest.y)*dist_robot;
        traj.push_back({dest, angle, {0., dest.y}, 2});

        dest = playground[pgy].project(lastPos);
        dest.y += SIGN(lastPos.y - dest.y)*R_ROBOT;
        traj.push_back({dest, angle, {0,0}, 0});
    }

    traj.push_back({destPt, destAngle, {0,0}, 3});
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


