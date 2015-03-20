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

void trajmngr_init(trajectory_manager_t* tm, const int32_t mat_rob2pods[NB_PODS][NB_SPDS]) {
    // will set the indexes to 0 and the slots' state to empty
    memset(tm, 0, sizeof(*tm));

    trajctlr_init(&tm->ctlr, mat_rob2pods);
}

void trajmngr_reset(trajectory_manager_t* tm) {
    trajctlr_reset(&tm->ctlr);
}

int _new_traj_slot(trajectory_manager_t* tm, uint16_t idx) {
    int error = 0;
    sTrajSlot_t* ts = &tm->slots[idx];

    switch(tm->state) {
    case TM_STATE_WAIT_TRAJ:
        if(ts->sid == 0) {
            tm->curr_tid = ts->tid;
            tm->next_sid = 1;
            tm->curr_element = idx << 1;

            tm->gx = ts->p1_x;
            tm->gy = ts->p1_y;
            tm->gtheta = ts->seg_start_theta;

            tm->state = TM_STATE_WAIT_START;
        }
        break;

    case TM_STATE_WAIT_START:
    case TM_STATE_FOLLOWING:
        if(ts->tid == tm->curr_tid) {
            if(ts->sid == tm->next_sid) {
                tm->next_sid = (tm->next_sid + 1)&31;
            }
        }
        else {
            if(ts->sid == 0) {
                tm->curr_tid = ts->tid;
                tm->next_sid = 1;
                tm->curr_element = idx << 1;

                tm->gx = ts->p1_x;
                tm->gy = ts->p1_y;
                tm->gtheta = ts->seg_start_theta;

                tm->state = TM_STATE_WAIT_START;
            }
        }
        break;
    }

    return error;
}

int trajmngr_new_traj_el(trajectory_manager_t* tm, const sTrajOrientElRaw_t *te) {
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

    int nb = trajslot_create_from_msg(te, s1, s2);

    error = error?: _new_traj_slot(tm, tm->slots_insert_idx);
    if(nb > 1) {
        error = error?: _new_traj_slot(tm, (tm->slots_insert_idx + 1) % TRAJ_MAX_SLOTS);
    }

    tm->slots_insert_idx = (tm->slots_insert_idx + nb) % TRAJ_MAX_SLOTS;

    return error;
}

int trajmngr_update(trajectory_manager_t* tm) {
    int x_sp = tm->gx;
    int y_sp = tm->gy;
    int theta_sp = tm->gtheta;
    uint32_t t = bn_intp_micros2s(micros());

    trajctlr_begin_update(&tm->ctlr);

    switch(tm->state) {
    default:
    case TM_STATE_WAIT_TRAJ:
        x_sp = tm->gx;
        y_sp = tm->gy;
        theta_sp = tm->gtheta;
        break;

    case TM_STATE_WAIT_START:
        {
            sTrajSlot_t* s = &tm->slots[tm->curr_element >> 1];

            if(t <= s->seg_start_date) {
                x_sp = tm->gx;
                y_sp = tm->gy;
                theta_sp = tm->gtheta;
                break;
            }
        }
        /* no break */
    case TM_STATE_FOLLOWING:
        {
            typeof(tm->cache)* sc = &tm->cache;
            sTrajSlot_t* s;
            sTrajSlot_t* s_next;

            do {
                // get current slot
                s = &tm->slots[tm->curr_element >> 1];
#ifdef ARCH_X86_LINUX
                assert(s->state != SLOT_EMPTY && s->tid == tm->curr_tid);
#endif

                // get next slot and check its consistency
                s_next = &tm->slots[((tm->curr_element >> 1) + 1)%TRAJ_MAX_SLOTS];
                if(s_next->state == SLOT_EMPTY || s_next->tid != s->tid || s_next->sid != ((s->sid + 1)&31)) {
                    s_next = NULL;
                }

                if(tm->curr_element&1) { // following arc
                    if(!s_next || s->is_last_element) {
                        tm->state = TM_STATE_WAIT_TRAJ;

                        x_sp = tm->gx = s->p2_x;
                        y_sp = tm->gy = s->p2_y;
                        theta_sp = tm->gtheta = s->arc_start_theta;
                        break;
                    }
                    else {
#ifdef ARCH_X86_LINUX
                        assert(t >= s->arc_start_date);
#endif

                        // switch to next element's segment
                        if(t >= s_next->seg_start_date) {
                            tm->curr_element++;
                            continue;
                        }
                    }
                }
                else { // following segment
#ifdef ARCH_X86_LINUX
                    assert(t >= s->seg_start_date);
#endif

                    // switch to arc
                    if(t >= s->arc_start_date) {
                        tm->curr_element++;
                        continue;
                    }
                }
            } while(0);

            if(tm->state == TM_STATE_WAIT_TRAJ) {
                break;
            }

            // check if next element may be used to update current one
            if(s->state == SLOT_WAITING_NEXT && s_next) {
                trajslot_update_with_next(s, s_next);
            }

#ifdef ARCH_X86_LINUX
            assert(
                    (s->state == SLOT_WAITING_NEXT && !s_next) ||
                    (s->state == SLOT_OK && !s_next && s->is_last_element) ||
                    (s->state == SLOT_OK && s_next && s_next->state != SLOT_EMPTY)
            );
            assert(s->arc_start_date >= s->seg_start_date);
#endif

            // check cache consistency
            if(sc->id != s->id || ((tm->curr_element&1) && sc->state != TM_CACHE_STATE_ARC && s_next) || (!(tm->curr_element&1) && sc->state != TM_CACHE_STATE_LINE)) {
                sc->id = s->id;
                sc->state = (tm->curr_element&1) ? TM_CACHE_STATE_ARC : TM_CACHE_STATE_LINE;

                switch(sc->state){
                case TM_CACHE_STATE_ARC:
                    // compute angular speed
                    sc->arc.omega_z = (((int64_t)s->arc_spd << (RAD_SHIFT + SHIFT)) / (int64_t)s->c_r);

                    // compute arc total duration in periods
                    sc->arc.dur = (int32_t)(s_next->seg_start_date - s->arc_start_date) / USpP;
                    break;
                case TM_CACHE_STATE_LINE:
                    {
                        // compute (co)sinus of line
                        int cl = ((int64_t) (s->p2_x - s->p1_x) << SHIFT) / s->seg_len; // (<< SHIFT)
                        int sl = ((int64_t) (s->p2_y - s->p1_y) << SHIFT) / s->seg_len; // (<< SHIFT)

                        // compute desired speed for x and y
                        sc->line.spd_x = ((int64_t) s->seg_spd * (int64_t) cl) >> SHIFT; // (IpP << SHIFT)
                        sc->line.spd_y = ((int64_t) s->seg_spd * (int64_t) sl) >> SHIFT; // (IpP << SHIFT)

                        // compute line total duration in periods
                        sc->line.dur = (int32_t)(s->arc_start_date - s->seg_start_date) / USpP;
                    }
                case TM_CACHE_STATE_EMPTY:
                    break;
                }
            }

            int dur; // duration since start of element (in P)
            int elt_dur; // total duration of element (in P)
            int dtheta; // delta theta for the element (theta_end - theta_start) (in rad << (RAD_SHIFT + SHIFT))
            int dir; // direction of rotation (1: negative angle / ClockWise, 0: positive angle / CounterClockWise)
            int theta0; // initial orientation at start of element (in rad << (RAD_SHIFT + SHIFT))

            // TODO check if we are too far from the assumed current position
            // TODO handle deceleration before segment or arc

            if(tm->curr_element & 1) { // following the arc
#ifdef ARCH_X86_LINUX
                assert(s_next);
                assert(sc->id == s->id && sc->state == TM_CACHE_STATE_ARC);
#endif

                // Compute the delta of time on the arc
                dur = t - s->arc_start_date;
                dur += PER_ASSER; // to anticipate the position (might be updated later if doesn't anticipate enough)
                dur /= USpP; // convert duration to periods (from microseconds)

                int alpha = -dur * sc->arc.omega_z; // (in rad << (RAD_SHIFT + SHIFT))

                int ca = COS(alpha); // (<< SHIFT)
                int sa = SIN(alpha); // (<< SHIFT)

                int vec_x = s->p2_x - s->c_x; // (in I << SHIFT)
                int vec_y = s->p2_y - s->c_y; // (in I << SHIFT)

                int vec2_x = ((int64_t)vec_x * (int64_t)ca - (int64_t)vec_y * (int64_t)sa) >> SHIFT;
                int vec2_y = ((int64_t)vec_x * (int64_t)sa + (int64_t)vec_y * (int64_t)ca) >> SHIFT;

                x_sp = s->c_x + vec2_x; // (in I << SHIFT)
                y_sp = s->c_y + vec2_y; // (in I << SHIFT)

                // compute delta theta between begin and end of line
                dir = s->rot2_dir;
                dtheta = s_next->seg_start_theta - s->arc_start_theta; // (rad << (RAD_SHIFT + SHIFT))
                theta0 = s->arc_start_theta;

                // compute segment duration
                elt_dur = sc->arc.dur; // in periods
            }
            else { // following the straight line
#ifdef ARCH_X86_LINUX
                assert(sc->id == s->id && sc->state == TM_CACHE_STATE_LINE);
#endif

                // compute current duration from start of line
                dur = t - s->seg_start_date;
                dur += PER_ASSER; // to anticipate the position (might be updated later if doesn't anticipate enough)
                dur /= USpP; // convert duration to periods (from microseconds)

                // compute next position set point (get speed along x and y in cache to avoid storing it in tm and calculating it each loop)
                x_sp = s->p1_x + sc->line.spd_x * dur;
                y_sp = s->p1_y + sc->line.spd_y * dur;

                // compute delta theta between begin and end of line
                dir = s->rot1_dir;
                dtheta = s->arc_start_theta - s->seg_start_theta; // (rad << (RAD_SHIFT + SHIFT))
                theta0 = s->seg_start_theta;

                // compute segment duration (get from cache)
                elt_dur = sc->line.dur; // in periods
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

    trajctlr_end_update(&tm->ctlr, x_sp, y_sp, theta_sp);

    return 0;
}

/** Description:
 * Store and convert the position and heading in robot units received by "bn_received function"
 * If the robot is motionless, the goal of robot is actualized
 */
void trajmngr_set_pos(trajectory_manager_t* tm, const sPosPayload *pos) {
    if(pos->id == ELT_PRIMARY) { // Keep information for primary robot
        int x, y, theta;

        x = isD2I(pos->x); // (I << SHIFT)
        y = isD2I(pos->y); // (I << SHIFT)
        theta = isROUND(D2I(WDIAM)*pos->theta); // (I.rad << SHIFT)

        trajctlr_set_pos(&tm->ctlr, x, y, theta);

        if(tm->state == TM_STATE_WAIT_TRAJ) {
            tm->gx = x;
            tm->gy = y;
            tm->gtheta = theta;
        }
    }
}
