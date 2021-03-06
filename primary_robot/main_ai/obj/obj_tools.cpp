/*
 * obj_tools.cpp
 *
 *  Created on: 12 févr. 2015
 *      Author: seb
 */

#include <ai_tools.h>

#include <iostream>

#include "types.h"
#include <obj.h>
#include <obj_tools.h>
#include <tools.h>

//#define DEBUG_OBJ

using namespace std;

void updateEndTraj(float theta, Point2D<float> *pt, float r, vector<astar::sObs_t>& obs) {
    int i, N = obs.size();
    for (i = 1; i < 4; i++) {
        obs[N - i - 1].c.x = pt->x + (r) * cos(theta + i * M_PI_2);
        obs[N - i - 1].c.y = pt->y + (r) * sin(theta + i * M_PI_2);
        obs[N - i - 1].active = 1;
        obs[N - i - 1].r = r - 0.5;
    }
}

void updateNoHaftTurn(float theta, Point2D<float>& pt, vector<astar::sObs_t>& obs) {
    int i, N = obs.size();
    sNum_t r, speed = statuses.getLastSpeed(ELT_PRIMARY);
    r = speed / 3;
    if (r > 15)
        r = 15;

    for (i = 1; i < 4; i++) {
        obs[N - i - 4].c.x = pt.x + (r) * cos(theta * M_PI / 180 + i * M_PI_2);
        obs[N - i - 4].c.y = pt.y + (r) * sin(theta * M_PI / 180 + i * M_PI_2);
        obs[N - i - 4].active = 1;
        obs[N - i - 4].r = r - 0.5;
    }
    if (r < 0.1) {
        for (i = 1; i < 4; i++)
            obs[N - i - 4].active = 0;
    }
}

void printEndTraj(int /*N*/) {
    /*
    obs_updated[N - 2]++;
    obs_updated[N - 3]++;
    obs_updated[N - 4]++;
    */
}

/*
 * Update the destination point in the obstacle list and the selected point.
 * And update the display on the screen.
 * The second argument is used for a non holonomic robot with the 3 circles anti half turn.
 */
void loadingPath(sPath_t /*_path*/, int /*num*/) {
    //path = _path; //FIXME
/*
#if !HOLONOMIC
    if (num >= 0) {
        sPt_t _ep = listObj[num]->getDestPoint();
        float  angle = listObj[num]->getDestPointOrient();

        updateEndTraj(angle, &_ep, 5);
        printEndTraj();
    }
    else
        printf("Error with the second parameter in loadingPath\n");
#endif
*/
   // pt_select = _path.path[_path.path_len - 1].p2;
   // obs[N - 1].c = pt_select;
   // obs_updated[N - 1]++;
}

/*
 * Positions of robots must be already updated in obs
 */
int nextObj(const unsigned int start_time, const int robot, const bool axle, paramObj par) {
    float tmp_val = 0.;
    float tmp_val2;
    int tmp_inx = -1; //index of the objective will be selected
    int N = par.obs.size();
    Point2D<float> pos_robot(par.obs[robot].c.x, par.obs[robot].c.y);

    logs << INFO << "Starting NextObj";

    // Active the robot and the destination point
    par.obs[robot].active = 1;
    par.obs[par.obs.size()-1].active = 1;

    for (unsigned int i = 0 ; i < par.obj.size() ; i++) {  // Search the best objective
        if (par.obj[i]->getState() != ACTIVE)
            continue;

        if (par.obj[i]->update(axle, par.obs, robot) < 0) {
#ifdef DEBUG_OBJ
            logs << DEBUG << "No find path to achieve the objective for objective n°" << i;
#endif
            continue;
        }

        if(par.obj[i]->updateDestPointOrient(par) < 0){
#ifdef DEBUG_OBJ
            logs << DEBUG << "No actuator available for objective n°" << i;
#endif
            continue;
        }

        tmp_val2 = par.obj[i]->getYield(start_time);
#ifdef DEBUG_OBJ
        logs << DEBUG << "objectif n°" << i << " with ratio=" << tmp_val2;
#endif
        if (tmp_val2 > tmp_val && par.obj[i]->getDestPointOrient() != -1) {  // Update best objective
            tmp_val = tmp_val2;
            tmp_inx = i;
        }
    }  // End of search the best objective

    if (tmp_inx >= 0) {  // Update end of trajectory
        loadingPath(par.obj[tmp_inx]->getPath(), tmp_inx); //FIXME
        par.obs[N - 1].c = {par.obj[tmp_inx]->getDestPoint().x, par.obj[tmp_inx]->getDestPoint().y};
        par.obsUpdated[N - 1]++;
    }

    logs << INFO << "State of objectives :";
    for(Obj* i : par.obj)  // Print the all the objective list
        i->print();

    logs << INFO << "State of actuators :";
    for(Actuator const& i : par.act){
        logs << INFO << "type=" << i.type
                     << "; id=" << i.id;

        if(i.type == SANDDOOR){
            logs << ": FULL ?" << i.doors.full;
        }
//        else if(i.type == S){
//            logs << "; distributor=" << i.cupActuator.distributor
//                 << "; full=" << i.cupActuator.full;
//        }
    }

    logs << INFO << "The selected objective is :" << tmp_inx;

    return (tmp_inx);
}



int metObj(int numObj, paramObj par){
    static bool first = true;

    if(numObj < 0 || numObj > (int) par.obj.size()){
        logs << ERR << "metObj, bad numObj=" << numObj;
        return -1;
    }

    if(first){
        par.obj[numObj]->initObj(par);
        first = false;
        logs << INFO << "Starting objective number : " << numObj;
    }else{
        if(par.obj[numObj]->loopObj(par) == 0){ //0 finished
            first = true;
            logs << INFO << "Ending objective number : " << numObj;
            vector <unsigned int> num = par.obj[numObj]->getNumObs();
            for(unsigned int i : num){
                par.obs[i].active = 0;
                par.obsUpdated[i]++;
            }
            return 0;
        }
    }

    return 1;
}
