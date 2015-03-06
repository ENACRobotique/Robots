/*
 * trajectory_manager.c
 *
 *  Created on: 22 févr. 2015
 *      Author: ludo6431, Yoyo
 *  Description:
 *  The trajectory manager allow to process some type of messages like
 *      a new speed set point
 *      a new elements of trajectory
 *      a new current position of the robot
 *  These messages are convert or adapted in units robots before to be stock or take into account
 *
 */

#include <string.h>
#ifdef ARCH_X86_LINUX
#include <assert.h>
#endif

#include "params.h"

#include "trajectory_manager.h"

void _updateSlotKnowingNext(sTrajSlot_t* curr, sTrajSlot_t* next);
int _convertMsg2Slots(sTrajOrientElRaw_t* m, sTrajSlot_t* s1, sTrajSlot_t* s2);

void trajmngr_init(trajectory_manager_t* tm, trajectory_controller_t* tc) {
    // will set the indexes to 0 and the slots' state to empty
    memset(tm, 0, sizeof(*tm));

    tm->ctlr = tc;
}

int trajmngr_new_traj_slot(trajectory_manager_t* tm, sTrajSlot_t* ts) {
    int error = 0;

    switch(tm->state) {
    case S_WAIT:
        // TODO
        break;

    case S_FOLLOWING:
        if(ts->tid == tm->curr_tid) {
            if(ts->sid == tm->next_sid) {
                tm->next_sid = (tm->next_sid + 1)&15;
            }
            else if((ts->sid + 1)&15 == tm->next_sid) {
                // received at least twice the same message, ignoring
            }
            else {
                error = -1;
            }
        }
        // TODO...
        break;
    }

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

    return -1;
}

int trajmngr_new_traj_el(trajectory_manager_t* tm, sTrajOrientElRaw_t *te) {
    /* Description:
     * If a valid trajectory element is received thus the information within the message are
     * stored in one of the two array following the current or the next trajectory
     * Return a negative value if an error occurs and 0 if no error
     */
    int error = 0;

    sTrajSlot_t* s1 = &tm->slots[tm->slots_insert_idx];
    sTrajSlot_t* s2 = &tm->slots[(tm->slots_insert_idx + 1) % TRAJ_MAX_SLOTS];

    if(s1->state != SLOT_EMPTY || s2->state != SLOT_EMPTY) {
        return -1; // no more empty slots
    }

    int nb = _convertMsg2Slots(te, s1, s2);
    tm->slots_insert_idx = (tm->slots_insert_idx + nb) % TRAJ_MAX_SLOTS;

    error = error?: trajmngr_new_traj_slot(tm, s1);
    if(nb > 1) {
        error = error?: trajmngr_new_traj_slot(tm, s2);
    }

    return error;
}

/** Description:
 * Store and convert the position and heading in robot units received by "bn_received function"
 * If the robot is motionless, the goal of robot is actualized
 */
void trajmngr_new_pos(trajectory_manager_t* tm, sPosPayload *pos) {
    if (pos->id == ELT_PRIMARY) { // Keep information for primary robot
        tm->ctlr->x = isD2I(pos->x); // (I << SHIFT)
        tm->ctlr->y = isD2I(pos->y); // (I << SHIFT)
        tm->ctlr->theta = isROUND(D2I(WDIAM)*pos->theta); // (I.rad << SHIFT)

        if (tm->state == S_WAIT) {
            tm->ctlr->gx = tm->ctlr->x;
            tm->ctlr->gy = tm->ctlr->y;
            tm->ctlr->gtheta = tm->ctlr->theta;
        }
    }
}

#define dCAST(v) ((double)(v))
#define dCSHIFT(s) ((double)(1 << (s)))
#define D2Isi(d, i) (D2I(dCAST(d) / dCSHIFT(i)))
#define isD2Isi(d, i) isROUND(D2Isi(d, i))
#define isDpS2IpPs5(dps) isROUND(DpS2IpP(dCAST(dps) / dCSHIFT(5)))
#define _SPDCALC(d, t) (int32_t)((t)>0 ? ((int64_t)(d) * (int64_t)USpP / (int64_t)(t)) : 0)
void _convertMsg2Slot(sTrajOrientElRaw_t* m, sTrajSlot_t* s, int8_t ssid) {
    s->tid = m->tid;
    s->sid = m->sid;
    s->ssid = ssid;

    s->seg_start_date = m->t + (ssid ? (m->dt1 + m->dt2) * 1000 : 0);
    s->seg_start_theta = isD2Isi(m->elts[ssid].theta1, 13);
    s->rot1_dir = m->elts[ssid].rot1_dir;
    s->p1_x = isD2Isi(m->elts[ssid].p1_x, 6);
    s->p1_y = isD2Isi(m->elts[ssid].p1_y, 6);
    s->p2_x = isD2Isi(m->elts[ssid].p2_x, 6);
    s->p2_y = isD2Isi(m->elts[ssid].p2_y, 6);
    s->seg_len = isD2Isi(m->elts[ssid].seg_len, 5);

    s->arc_start_date = m->t + (m->dt1 + (ssid ? m->dt2 + m->dt3 : 0)) * 1000;
    s->arc_start_theta = isD2Isi(m->elts[ssid].theta2, 13);
    s->rot2_dir = m->elts[ssid].rot2_dir;
    s->c_x = isD2Isi(m->elts[ssid].c_x, 6);
    s->c_y = isD2Isi(m->elts[ssid].c_y, 6);
    s->c_r = isD2Isi(m->elts[ssid].c_r, 5);
    s->arc_len = isD2Isi(m->elts[ssid].arc_len, 5);
    s->arc_spd = 0; // can't be computed here, will be updated in _updateSlotKnowingNext()

    // compute segment speed
    int32_t dt_us = s->arc_start_date - s->seg_start_date; // duration for segment item in microseconds
    s->seg_spd = _SPDCALC(s->seg_len, dt_us);

    if(m->elts[ssid].is_last_element) {
        s->state = SLOT_OK;
    }
    else {
        s->state = SLOT_WAITING_NEXT;
    }
}

void _updateSlotKnowingNext(sTrajSlot_t* curr, sTrajSlot_t* next) {
    if(
            curr->state == SLOT_WAITING_NEXT &&
            next->state != SLOT_EMPTY &&
            next->seg_start_date >= curr->arc_start_date &&
            next->tid == curr->tid &&
            (next->sid == curr->sid || next->sid == curr->sid + 1 || next->sid - 1 == curr->sid)
    ) {
        int32_t dt_us = next->seg_start_date - curr->arc_start_date; // duration for arc item in microseconds
        curr->arc_spd = _SPDCALC(curr->arc_len, dt_us);

        curr->state = SLOT_OK;
    }
}

int _convertMsg2Slots(sTrajOrientElRaw_t* m, sTrajSlot_t* s1, sTrajSlot_t* s2) {
    int ret = 1;

    _convertMsg2Slot(m, s1, 0);

    if (!m->elts[0].is_last_element) {
        ret = 2;

        _convertMsg2Slot(m, s2, 1);

        _updateSlotKnowingNext(s1, s2);
    }

    return ret;
}
