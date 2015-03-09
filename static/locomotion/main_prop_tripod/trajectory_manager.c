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
#include <millis.h>
#elif defined(ARCH_LPC21XX)
#include <sys_time.h>
#endif
#include "bn_intp.h"

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
        error = -1;
        break;

    case S_FOLLOWING:
        if(ts->tid == tm->curr_tid) {
            if(ts->sid == tm->next_sid) {
                tm->next_sid = (tm->next_sid + 1)&31;
            }
            else if(((ts->sid + 1)&31) == tm->next_sid) {
                // received at least twice the same message, ignoring
            }
            else {
                error = -1;
            }
        }
        else if(ts->tid != tm->curr_tid) {
            if(ts->sid == 0) {
                error = -1;
                // TODO
            }
            else {
                // did not receive the first element of a new trajectory
                error = -5;
            }
        }
        else {
            error = -1;
            // TODO
        }
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

    return error;
}

int trajmngr_update(trajectory_manager_t* tm) {
    int x_sp, y_sp, theta_sp;

    trajctlr_begin_update(tm->ctlr);

    switch(tm->state) {
    default:
    case S_WAIT:
        x_sp = tm->gx;
        y_sp = tm->gy;
        theta_sp = tm->gtheta;
        break;

    case S_FOLLOWING:
        {
            uint32_t t = bn_intp_micros2s(micros());
            sTrajSlot_t* s = &tm->slots[tm->curr_slot_idx >> 1];
            sTrajSlot_t* s_next = &tm->slots[((tm->curr_slot_idx >> 1) + 1)%TRAJ_MAX_SLOTS];

            if(s_next->state == SLOT_EMPTY || s_next->tid != s->tid || s_next->sid != ((s->sid + 1)&31)) {
                s_next = NULL;
            }

            if(s->state == SLOT_WAITING_NEXT && s_next) {
                _updateSlotKnowingNext(s, s_next);
            }

#ifdef ARCH_X86_LINUX
            assert(
                    (s->state == SLOT_WAITING_NEXT && !s_next) ||
                    (s->state == SLOT_OK && !s_next && s->is_last_element) ||
                    (s->state == SLOT_OK && s_next && s_next->state != SLOT_EMPTY)
            );
            assert(s->arc_start_date >= s->seg_start_date);
#endif

            // FIXME implements cache system stored in the trajectory_manager for once per element computations

            int dur; // duration since start of element (in P)
            int elt_dur; // total duration of element (in P)
            int dtheta; // delta theta for the element (theta_end - theta_start) (in rad << (RAD_SHIFT + SHIFT))
            int dir; // direction of rotation (1: negative angle, 0: positive angle)
            int theta0; // initial orientation at start of element (in rad << (RAD_SHIFT + SHIFT))

            if(tm->curr_slot_idx & 1) { // following the arc
                // Check if the next slot is present
                if(!s_next) {
                    tm->state = S_WAIT; // We wait if the next slot is not present yet
                    // FIXME handle case when message we are waiting for just arrived
                    // FIXME handle out of this function without theta_sp computation

                    x_sp = 0; // TODO ...
                    y_sp = 0; // TODO ...
                    theta_sp = 0; // TODO ...
                }
                else {
                    // Check if time is greater than the next start of the segment
                    if (t > s_next->seg_start_date) {
                        // TODO go to next step seg follow?
                        // check if position is consistent with next step seg start
                    }

                    // Compute the delta of time on the arc
                    dur = t - s->arc_start_date;
                    dur += PER_ASSER; // to anticipate the position (might be updated later if doesn't anticipate enough)
                    dur /= USpP; // convert duration to periods (from microseconds)

                    int alpha = -dur * (((int64_t)s->arc_spd << (RAD_SHIFT + SHIFT)) / (int64_t)s->c_r); // (in rad << (RAD_SHIFT + SHIFT))

                    // compute (co)sinus of the rope
                    int ca = COS(alpha); // (<< SHIFT)
                    int sa = SIN(alpha); // (<< SHIFT)

                    int vec_x = s->p2_x - s->c_x; // (in I << SHIFT)
                    int vec_y = s->p2_y - s->c_y; // (in I << SHIFT)

                    int vec2_x = ((int64_t)vec_x * (int64_t)ca - (int64_t)vec_y * (int64_t)sa) >> SHIFT;
                    int vec2_y = ((int64_t)vec_x * (int64_t)sa + (int64_t)vec_y * (int64_t)ca) >> SHIFT;

                    x_sp = s->c_x + vec2_x; // (in I << SHIFT)
                    y_sp = s->c_y + vec2_y; // (in I << SHIFT)

                    // compute delta theta between begin and end of line (XXX may be done once per segment)
                    dir = s->rot2_dir;
                    dtheta = s_next->seg_start_theta - s->arc_start_theta; // (rad << (RAD_SHIFT + SHIFT))
                    theta0 = s->arc_start_theta;

                    // compute segment duration (XXX may be done once per segment)
                    elt_dur = s_next->seg_start_date - s->arc_start_date; // in microseconds
                    elt_dur /= USpP; // in periods
                }
            }
            else { // following the straight line
                if(t > s->arc_start_date) {
                    // TODO go to arc follow?
                    // check if position is consistent with arc start
                }

                // compute current duration from start of line
                dur = t - s->seg_start_date;
                dur += PER_ASSER; // to anticipate the position (might be updated later if doesn't anticipate enough)
                dur /= USpP; // convert duration to periods (from microseconds)

                // compute (co)sinus of line (XXX may be done once per segment)
                int cl = ((int64_t) (s->p2_x - s->p1_x) << SHIFT) / s->seg_len; // (<< SHIFT)
                int sl = ((int64_t) (s->p2_y - s->p1_y) << SHIFT) / s->seg_len; // (<< SHIFT)

                // compute desired speed for x and y (XXX may be done once per segment)
                int vx = ((int64_t) s->seg_spd * (int64_t) cl) >> SHIFT; // (IpP << SHIFT)
                int vy = ((int64_t) s->seg_spd * (int64_t) sl) >> SHIFT; // (IpP << SHIFT)

                // compute next position set point
                x_sp = s->p1_x + vx * dur;
                y_sp = s->p1_y + vy * dur;

                // compute delta theta between begin and end of line (XXX may be done once per segment)
                dir = s->rot1_dir;
                dtheta = s->arc_start_theta - s->seg_start_theta; // (rad << (RAD_SHIFT + SHIFT))
                theta0 = s->seg_start_theta;

                // compute segment duration (XXX may be done once per segment)
                elt_dur = s->arc_start_date - s->seg_start_date; // in microseconds
                elt_dur /= USpP; // in periods
            }

            // ensures dtheta is of the right sign
            if(dir) {
                while(dtheta > 0) {
                    dtheta -= (issPI << 1 /* 2*PI */); // if dir==1, dtheta must be negative
                }
            }
            else {
                while(dtheta < 0) {
                    dtheta += (issPI << 1 /* 2*PI */); // if dir==0, dtheta must be positive
                }
            }

            // compute the next desired orientation (same calculation for seg and arc)
            theta_sp = theta0 + (int64_t)dtheta * (int64_t)dur / elt_dur;

            break;
        }
    }

    trajctlr_end_update(tm->ctlr, x_sp, y_sp, theta_sp);

    return 0;
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
    if(pos->id == ELT_PRIMARY) { // Keep information for primary robot
        tm->ctlr->x = isD2I(pos->x); // (I << SHIFT)
        tm->ctlr->y = isD2I(pos->y); // (I << SHIFT)
        tm->ctlr->theta = isROUND(D2I(WDIAM)*pos->theta); // (I.rad << SHIFT)

        if(tm->state == S_WAIT) {
            tm->gx = tm->ctlr->x;
            tm->gy = tm->ctlr->y;
            tm->gtheta = tm->ctlr->theta;
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
    s->sid = (m->sid << 1) + ssid;
    s->is_last_element = m->elts[ssid].is_last_element;

    s->seg_start_date = m->t + (ssid ? (m->dt1 + m->dt2) * 1000 : 0);
#if RAD_SHIFT + SHIFT > 13
    s->seg_start_theta = m->elts[ssid].theta1 << (RAD_SHIFT + SHIFT - 13);
#else
    s->seg_start_theta = m->elts[ssid].theta1 >> (13 - (RAD_SHIFT + SHIFT));
#endif
    s->rot1_dir = m->elts[ssid].rot1_dir;
    s->p1_x = isD2Isi(m->elts[ssid].p1_x, 6);
    s->p1_y = isD2Isi(m->elts[ssid].p1_y, 6);
    s->p2_x = isD2Isi(m->elts[ssid].p2_x, 6);
    s->p2_y = isD2Isi(m->elts[ssid].p2_y, 6);
    s->seg_len = isD2Isi(m->elts[ssid].seg_len, 5);

    s->arc_start_date = m->t + (m->dt1 + (ssid ? m->dt2 + m->dt3 : 0)) * 1000;
#if RAD_SHIFT + SHIFT > 13
    s->arc_start_theta = m->elts[ssid].theta2 << (RAD_SHIFT + SHIFT - 13);
#else
    s->arc_start_theta = m->elts[ssid].theta2 >> (13 - (RAD_SHIFT + SHIFT));
#endif
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
            next->sid == ((curr->sid + 1)&31)
    ) {
        int32_t dt_us = next->seg_start_date - curr->arc_start_date; // duration for arc item in microseconds
        curr->arc_spd = _SPDCALC(curr->arc_len, dt_us);

        curr->state = SLOT_OK;
    }
}

int _convertMsg2Slots(sTrajOrientElRaw_t* m, sTrajSlot_t* s1, sTrajSlot_t* s2) {
    int ret = 1;

    _convertMsg2Slot(m, s1, 0);

    if(!m->elts[0].is_last_element) {
        ret = 2;

        _convertMsg2Slot(m, s2, 1);

        _updateSlotKnowingNext(s1, s2);
    }

    return ret;
}
