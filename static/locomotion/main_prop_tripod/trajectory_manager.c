/*
 * trajectory_manager.c
 *
 *  Created on: 22 févr. 2015
 *      Author: ludo6431
 */

#include <string.h>

#include "params.h"

#include "trajectory_manager.h"

void trajmngr_init(trajectory_manager_t* tm) {
    memset(tm, 0, sizeof(*tm));
    //TODO
}

int trajmngr_new_traj_el(trajectory_manager_t* tm, sTrajOrientElRaw_t *te) {
    /* Description:
     * If a valid trajectory element is received thus the information within the message are
     * stored in one of the two array following the current or the next trajectory
     * Return a negative value if an error occurs and 0 if no error
     */
    int error = 0;

//    // We received some step of the current trajectory that we are following
//    if (curr_traj_insert_sid > 0 && te->tid == curr_tid) {
//        // Enough size in the array to add more trajectory elements
//        if (curr_traj_insert_sid < TRAJ_MAX_SLOTS) {
//            // Expected follow up
//            if (te->sid == curr_traj_insert_sid) {
//                memcpy(&traj[curr_traj][curr_traj_insert_sid].raw, te, sizeof(sTrajElRaw_t));
//                traj[curr_traj][curr_traj_insert_sid].is_ok = 0;
//                curr_traj_insert_sid++;
//                state = S_RUN_TRAJ; // Follow the trajectory
//            }
//            else if (te->sid < curr_traj_insert_sid) {
//                // Step already received (no error, could be caused by duplication in the network)
//            }
//            else {
//                error = -1; // TODO error: bad step => invalidate all trajectory and ask new one
//            }
//        }
//        // Too much trajectory steps received
//        else {
//            error = -2; // TODO
//        }
//    }
//    // We received some step of the next trajectory, but we didn't switch to those (next tid is valid)
//    else if (next_traj_insert_sid > 0 && te->tid == next_tid) {
//        // Enough size in the array to add more trajectory elements
//        if (next_traj_insert_sid < TRAJ_MAX_SLOTS) {
//            memcpy(&traj[!curr_traj][next_traj_insert_sid].raw, te, sizeof(sTrajElRaw_t));
//            traj[!curr_traj][next_traj_insert_sid].is_ok = 0;
//            next_traj_insert_sid++;
//
//            state = S_CHG_TRAJ; // New trajectory to follow
//        }
//        else if (te->sid < next_traj_insert_sid) {
//            // Step already received (no error, could be caused by duplication in the network)
//        }
//        else {
//            error = -3; // TODO error: bad step => invalidate all trajectory and ask new one
//        }
//    }
//    // We received the first step of a new trajectory
//    else {
//        if (te->sid == 0) {
//            next_traj_insert_sid = 0; // Reset the index for the next trajectory
//            next_tid = te->tid; // get the next tid
//            memcpy(&traj[!curr_traj][next_traj_insert_sid].raw, te, sizeof(sTrajElRaw_t));
//            traj[!curr_traj][next_traj_insert_sid].is_ok = 0;
//            next_traj_insert_sid++;
//
//            state = S_CHG_TRAJ;
//        }
//        else {
//            error = -5; // TODO
//        }
//    }
//
//    // Processing of the error
//    if (error) {
//        // TODO
//    }

    return error;
}

void trajmngr_new_pos(trajectory_manager_t* tm, sPosPayload *pos) {
    /* Description:
     * Store and convert the position and heading in robot units
     * If the robot is motionless, the goal of robot is actualize
     */
    if (pos->id == ELT_PRIMARY) { // Keep information for primary robot
        tm->x = isD2I(pos->x); // (I << SHIFT)
        tm->y = isD2I(pos->y); // (I << SHIFT)
        tm->theta = isROUND(D2I(WDIAM)*pos->theta); // (I.rad << SHIFT)

        if (tm->state == S_WAIT) {
            tm->gx = tm->x;
            tm->gy = tm->y;
            tm->gtheta = tm->theta;
        }
    }
}

#define fCAST(v) ((float)(v))
#define dCAST(v) ((double)(v))
#define fCSHIFT(s) ((float)(1 << (s)))
#define dCSHIFT(s) ((double)(1 << (s)))
#define isD2Is5(d) isROUND(D2I(fCAST(d) / fCSHIFT(5)))
#define isD2Is6(d) isROUND(D2I(fCAST(d) / fCSHIFT(6)))
#define isD2Is13(d) isROUND(D2I(fCAST(d) / fCSHIFT(13)))
#define isDpS2IpPs5(dps) isROUND(DpS2IpP(fCAST(dps) / fCSHIFT(5)))
void _convertMsg2Slot(sTrajOrientElRaw_t* m, sTrajSlot_t* s, int8_t ssid) {
    s->tid = m->tid;
    s->sid = m->sid;
    s->ssid = ssid;
    s->sssid = 0;

    s->seg_start_date = m->t + (ssid ? (m->dt1 + m->dt2) * 1000 : 0);
    s->seg_start_theta = isD2Is13(m->elts[ssid].theta1);
    s->rot1_dir = m->elts[ssid].rot1_dir;
    s->p1_x = isD2Is6(m->elts[ssid].p1_x);
    s->p1_y = isD2Is6(m->elts[ssid].p1_y);
    s->p2_x = isD2Is6(m->elts[ssid].p2_x);
    s->p2_y = isD2Is6(m->elts[ssid].p2_y);
    s->seg_len = isD2Is5(m->elts[ssid].seg_len);
    s->seg_spd = 0; //TODO

    s->arc_start_date = m->t + (m->dt1 + (ssid ? m->dt2 + m->dt3 : 0)) * 1000;
    s->arc_start_theta = isD2Is13(m->elts[ssid].theta2);
    s->rot2_dir = m->elts[ssid].rot2_dir;
    s->c_x = isD2Is6(m->elts[ssid].c_x);
    s->c_y = isD2Is6(m->elts[ssid].c_y);
    s->c_r = isD2Is6(m->elts[ssid].c_r);
    s->arc_len = isD2Is5(m->elts[ssid].arc_len);
    s->arc_spd = 0; //TODO
}

int _convertMsg2Slots(sTrajOrientElRaw_t* m, sTrajSlot_t* s1, sTrajSlot_t* s2) {
    int ret = 1;

    _convertMsg2Slot(m, s1, 0);

    if (1 /* TODO */) {
        ret = 2;

        _convertMsg2Slot(m, s2, 1);
    }

    return ret;
}
