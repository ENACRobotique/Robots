/*
 * obj_tools.cpp
 *
 *  Created on: 12 févr. 2015
 *      Author: seb
 */


#include <ai_tools.h>
#include "types.h"
#include "ai.h"
#include <obj.h>
#include <obj_tools.h>
#include "math_ops.h"
#include "clap.h"

#include <iostream>

extern "C"{
#include <malloc.h>
}


using namespace std;



void updateEndTraj(sNum_t theta, sPt_t *pt, sNum_t r) {
    int i;
    for (i = 1; i < 4; i++) {
        obs[N - i - 1].c.x = pt->x + (r) * cos(theta * M_PI / 180 + i * M_PI_2);
        obs[N - i - 1].c.y = pt->y + (r) * sin(theta * M_PI / 180 + i * M_PI_2);
        obs[N - i - 1].active = 1;
        obs[N - i - 1].r = r - 0.5;
    }
}

void updateNoHaftTurn(sNum_t theta, sPt_t *pt) {
    int i;
    sNum_t r;
    r = speed / 3;
    if (r > 15)
        r = 15;

    for (i = 1; i < 4; i++) {
        obs[N - i - 4].c.x = pt->x + (r) * cos(theta * M_PI / 180 + i * M_PI_2);
        obs[N - i - 4].c.y = pt->y + (r) * sin(theta * M_PI / 180 + i * M_PI_2);
        obs[N - i - 4].active = 1;
        obs[N - i - 4].r = r - 0.5;
    }
    if (r < 0.1) {
        for (i = 1; i < 4; i++)
            obs[N - i - 4].active = 0;
    }
}

void printEndTraj() {
    obs_updated[N - 2]++;
    obs_updated[N - 3]++;
    obs_updated[N - 4]++;
}

/*
 * Update the destination point in the obstacle list and the selected point.
 * And update the display on the screen.
 * The second argument is used for a non holonomic robot with the 3 circles anti half turn.
 */
void loadingPath(sPath_t _path, int num) {
    path = _path;

#ifdef NON_HOLONOMIC
    if (num > 0) {
        sObjPt_t _ep = listObj[num]->entryPoint(listObj[num]->EP());
        updateEndTraj(_ep.angleEP, &_ep.c, _ep.radiusEP);
        printEndTraj();
    }
    else
        printf("Error with the second parameter in loadingPath\n");
#endif

    pt_select = _path.path[_path.path_len - 1].p2;
    obs[N - 1].c = pt_select;
    obs_updated[N - 1]++;
}

/*
 * Generate a new path with a obstacle list
 */
void set_traj(sPath_t *p, iABObs_t l[], int nb) {
    int i;
    p->path = (sTrajEl_t *) calloc(nb - 1, sizeof(sTrajEl_t));
    p->path_len = nb - 1;
    // p->dist // TODO
    // p->path_len // TODO

    for (i = 0; i < nb - 1; i++) {
        sSeg_t *s = tgt(l[i], l[i + 1]);

        p->path[i].p1 = s->p1;
        p->path[i].p2 = s->p2;
        distPt2Pt(&s->p1, &s->p2, &p->path[i].seg_len);

        p->path[i].obs.active = 1;
        p->path[i].obs.c = obs[O(l[i + 1])].c;
        p->path[i].obs.moved = 1;
        p->path[i].obs.r = fabs(obs[O(l[i + 1])].r) * (1 - 2 * DIR(l[i + 1]));
        if (i == nb - 1) {
            p->path[i].arc_len = 0.;
        }
        else {
            p->path[i].arc_len = arc_len(l[i], l[i + 1], l[i + 2]);
        }

        p->path[i].sid = i;
    }
}

int same_obs(sObs_t *obs1, sObs_t *obs2) {
    printf("r1=%f r2=%f\n", obs1->r, obs2->r);
    return (obs1->r == obs2->r && obs1->c.x == obs2->c.x && obs1->c.y == obs2->c.y);
}

int same_traj(sPath_t *traj1, sPath_t *traj2) {
    unsigned int t1_ind = traj1->path_len;
    unsigned int t2_ind = traj2->path_len;

    if (traj1->path_len == 0 || traj2->path_len == 0)
        return 0;
    printf("same_t 1.0\n");

    while ((int) t1_ind > 0 && (int) t2_ind > 0) { //pb si un step est terminer
        printf("same_t 2.0\n");
        if (same_obs(&(traj1->path[t1_ind - 1].obs), &(traj2->path[t2_ind - 1].obs))) {
            t1_ind--;
            t2_ind--;
        }
        else
            return 0;
    }
    printf("same_t 3.0\n");
    if (!(same_obs(&(traj1->path[t1_ind].obs), &(traj2->path[t2_ind].obs))))
        return 0;

    if ((traj1->path[t1_ind].p2.x == traj2->path[t2_ind].p2.x) && (traj1->path[t1_ind].p2.y == traj2->path[t2_ind].p2.y))
        return 0;
    else
        printf("same_t 4.0\n");
    return 1;
}

/*
 * Return the difference of the path length with the previous path used for call of this function
 * and the current path.
 */
int checkCurrentPathLenght(sPath_t &path) {
    static sPath_t prev_path;
    int ret = -1;

#ifdef DEBUG
    cout << "checkCurrentPAthLenght : lenght_1 = " << path.path_len << " and lenght_2 = " << prev_path.path_len << endl;
#endif

    ret = same_traj(&path, &prev_path);
    prev_path = path;

    return ret;
}

int next_obj(void) {
    sNum_t tmp_val = 0.;
    sNum_t tmp_val2;
    int tmp_inx = -1; //index of the objective will be selected

    printf("Start next_obj()\n");

    obs[N - 1].active = 1;

    for (unsigned int i = 0; i < listObj.size(); i++) {
        if (listObj[i]->active() == false)
            continue; //test if  objective is still active //TODO remove if finished

#if DEBUG
        //printObsActive();
#endif

        if (listObj[i]->updateDistance(_current_pos) < 0) {
#if DEBUG
            printf("No find path to achieve the objective for objective n°%d\n\n", i);
#endif
            continue;
        }

        tmp_val2 = listObj[i]->value();
#if DEBUG
        printf("objectif n°%hhi avec ratio=%f \n\n", i, tmp_val2);
#endif

        if (tmp_val2 > tmp_val) {         //Update best objective
            tmp_val = tmp_val2;
            tmp_inx = i;
        }
    }

    if (tmp_inx >= 0) { //Update end of trajectory
        loadingPath(listObj[tmp_inx]->path());
    }

#if DEBUG
    printListObj();
    printf("Objectif sélectionné : %i\n\n", tmp_inx);
#endif

    return (tmp_inx);
}



int metObj(int numObj){
    static bool first = true;
    int ret;

     if(first){
         listObj[numObj]->initObj();
         first = false;
     }
     if( (ret = listObj[numObj]->loopObj()) == -1){
         cerr << "[Error] [obj_tools.cpp] Bad class" << endl;
         return -1;
     }
     if(ret == 0){ //0 finished
         first = true;
         return 0;
     }

    return 1;
}
