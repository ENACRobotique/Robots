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
#include "pos_uncertainty.h"

#include "params.h"

#include "trajectory_manager.h"

void trajmngr_init(trajectory_manager_t* tm, const int32_t mat_rob2pods[NB_PODS][NB_SPDS]) {
    // will set the indexes to 0 and the slots' state to empty
    memset(tm, 0, sizeof(*tm));

    posctlr_init(&tm->ctlr, mat_rob2pods);
}

void trajmngr_reset(trajectory_manager_t* tm) {
    posctlr_reset(&tm->ctlr);
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

    int s1_empty = s1->state == SLOT_EMPTY || s1->tid != tm->curr_tid || (tm->state != TM_STATE_WAIT_TRAJ && s1->sid < tm->slots[tm->curr_element >> 1].sid);
    int s2_empty = s2->state == SLOT_EMPTY || s2->tid != tm->curr_tid || (tm->state != TM_STATE_WAIT_TRAJ && s2->sid < tm->slots[tm->curr_element >> 1].sid);

    if(tm->state != TM_STATE_WAIT_TRAJ && (!s1_empty || !s2_empty)) {
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
    int x_sp = tm->ctlr.x;
    int y_sp = tm->ctlr.y;
    int theta_sp = tm->ctlr.theta;
    int vx_sp = 0;
    int vy_sp = 0;
    int oz_sp = 0;
    uint32_t t = bn_intp_micros2s(micros());
    int32_t dt;

    posctlr_begin_update(&tm->ctlr);

    switch(tm->state) {
    default:
    case TM_STATE_WAIT_TRAJ:
        x_sp = tm->gx;
        y_sp = tm->gy;
        theta_sp = tm->gtheta;
        vx_sp = 0;
        vy_sp = 0;
        oz_sp = 0;
        break;

    case TM_STATE_WAIT_START:
        {
            sTrajSlot_t* s = &tm->slots[tm->curr_element >> 1];

            dt = t - s->seg_start_date;
            if(dt <= 0) {
                x_sp = s->p1_x;
                y_sp = s->p1_y;
                theta_sp = s->seg_start_theta;
                vx_sp = 0;
                vy_sp = 0;
                oz_sp = 0;
                break;
            }

            // time is ok, let's go
            tm->state = TM_STATE_FOLLOWING;
        }
        /* no break */
    case TM_STATE_FOLLOWING:
        {
            typeof(tm->cache)* sc = &tm->cache;
            sTrajSlot_t* s;
            sTrajSlot_t* s_next;
            int quit;

            do {
                quit = 1;

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
                        vx_sp = 0;
                        vy_sp = 0;
                        oz_sp = 0;
                    }
                    else {
#ifdef ARCH_X86_LINUX
                        dt = t - s->arc_start_date;
                        assert(dt >= 0);
#endif

                        // switch to next element's segment
                        dt = t - s_next->seg_start_date;
                        if(dt >= 0) {
                            tm->curr_element = (tm->curr_element + 1)%(TRAJ_MAX_SLOTS<<1);

                            quit = 0; // one more loop
                        }
                    }
                }
                else { // following segment
#ifdef ARCH_X86_LINUX
                    dt = t - s->seg_start_date;
                    assert(dt >= 0);
#endif

                    // switch to arc
                    dt = t - s->arc_start_date;
                    if(dt >= 0) {
                        tm->curr_element = (tm->curr_element + 1)%(TRAJ_MAX_SLOTS<<1);

                        quit = 0; // one more loop
                    }
                }
            } while(!quit);

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
            dt = s->arc_start_date - s->seg_start_date;
            assert(dt >= 0);
#endif

            // check cache consistency
            if(sc->id != s->id || ((tm->curr_element&1) && sc->state != TM_CACHE_STATE_ARC && s_next) || (!(tm->curr_element&1) && sc->state != TM_CACHE_STATE_LINE)) {
                sc->id = s->id;
                sc->state = (tm->curr_element&1) ? TM_CACHE_STATE_ARC : TM_CACHE_STATE_LINE;

                switch(sc->state){
                case TM_CACHE_STATE_ARC:
                    // compute angular speed
                    if(s->c_r){
                        sc->arc.omega_z = -(((int64_t)s->arc_spd << (RAD_SHIFT + SHIFT)) / (int64_t)s->c_r);
                    }

                    // compute arc total duration in periods
                    dt = s_next->seg_start_date - s->arc_start_date;
                    sc->arc.dur = dt / USpP;
                    break;
                case TM_CACHE_STATE_LINE:
                    {
                        // compute (co)sinus of line
                        int cl = s->seg_len ? ((int64_t) (s->p2_x - s->p1_x) << SHIFT) / s->seg_len : 0; // (<< SHIFT)
                        int sl = s->seg_len ? ((int64_t) (s->p2_y - s->p1_y) << SHIFT) / s->seg_len : 0; // (<< SHIFT)

                        // compute desired speed for x and y
                        sc->line.spd_x = ((int64_t) s->seg_spd * (int64_t) cl) >> SHIFT; // (IpP << SHIFT)
                        sc->line.spd_y = ((int64_t) s->seg_spd * (int64_t) sl) >> SHIFT; // (IpP << SHIFT)

                        // compute line total duration in periods
                        dt = s->arc_start_date - s->seg_start_date;
                        sc->line.dur = dt / USpP;
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
                dur += USpP; // to anticipate the position (might be updated later if doesn't anticipate enough)
                dur /= USpP; // convert duration to periods (from microseconds)

                int alpha = dur * sc->arc.omega_z; // (in rad << (RAD_SHIFT + SHIFT))

                int ca = COS(alpha); // (<< SHIFT)
                int sa = SIN(alpha); // (<< SHIFT)

                int vec_x = s->p2_x - s->c_x; // (in I << SHIFT)
                int vec_y = s->p2_y - s->c_y; // (in I << SHIFT)

                int vec2_x = ((int64_t)vec_x * (int64_t)ca - (int64_t)vec_y * (int64_t)sa) >> SHIFT;
                int vec2_y = ((int64_t)vec_x * (int64_t)sa + (int64_t)vec_y * (int64_t)ca) >> SHIFT;

                x_sp = s->c_x + vec2_x; // (in I << SHIFT)
                y_sp = s->c_y + vec2_y; // (in I << SHIFT)

                vx_sp = (-(int64_t)sc->arc.omega_z * (int64_t)vec2_y) >> (RAD_SHIFT + SHIFT);
                vy_sp = ( (int64_t)sc->arc.omega_z * (int64_t)vec2_x) >> (RAD_SHIFT + SHIFT);

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
                dur += USpP; // to anticipate the position (might be updated later if doesn't anticipate enough)
                dur /= USpP; // convert duration to periods (from microseconds)

                // compute next position set point (get speed along x and y in cache to avoid storing it in tm and calculating it each loop)
                x_sp = s->p1_x + sc->line.spd_x * dur;
                y_sp = s->p1_y + sc->line.spd_y * dur;

                vx_sp = sc->line.spd_x;
                vy_sp = sc->line.spd_y;

                // compute delta theta between begin and end of line
                dir = s->rot1_dir;
                dtheta = s->arc_start_theta - s->seg_start_theta; // (rad << (RAD_SHIFT + SHIFT))
                theta0 = s->seg_start_theta;

                // compute segment duration (get from cache)
                elt_dur = sc->line.dur; // in periods
            }

            // ensures dtheta is of the right sign
            if(dir) {
                while(dtheta < 0) {
                    dtheta += (issPI << 1 /* 2*PI */); // if dir==1, dtheta must be positive
                }
            }
            else {
                while(dtheta > 0) {
                    dtheta -= (issPI << 1 /* 2*PI */); // if dir==0, dtheta must be negative
                }
            }

            // compute the next desired orientation (same calculation for seg and arc)
            theta_sp = elt_dur ? theta0 + (int64_t)dtheta * (int64_t)dur / elt_dur : theta0;

            oz_sp = elt_dur ? dtheta / elt_dur : 0;

            break;
        }
    }

    posctlr_end_update(&tm->ctlr, x_sp, y_sp, theta_sp, vx_sp, vy_sp, oz_sp);

    return 0;
}

/** Description:
 * Store and convert the position and heading in robot units received by "bn_received function"
 * If the robot is motionless, the goal of robot is actualized
 */
void trajmngr_set_pos(trajectory_manager_t* tm, const sGenericPosStatus *pos) {
    if(pos->id == ELT_PRIMARY) { // Keep information for primary robot
        int x, y, theta;
        int x_var, y_var, xy_var, theta_var;
        s2DPUncert_covar cov;

        gstatus2covar(pos, &cov);
        x = isD2I(cov.x); // (I << SHIFT)
        y = isD2I(cov.y); // (I << SHIFT)
        theta = iROUND(dASHIFT*cov.theta); // (rad << (RAD_SHIFT + SHIFT))
        x_var = iROUND(D2I(D2I(cov.a))*dVarPosSHIFT);
        y_var = iROUND(D2I(D2I(cov.c))*dVarPosSHIFT);
        xy_var = iROUND(D2I(D2I(cov.b))*dVarPosSHIFT);
        theta_var = iROUND(cov.d * dRadSHIFT * dRadSHIFT * dVarPosSHIFT);

        posctlr_set_pos(&tm->ctlr, x, y, theta);
        posctlr_set_pos_u(&tm->ctlr, x_var, y_var, xy_var, theta_var);

        if(tm->state == TM_STATE_WAIT_TRAJ) {
            tm->gx = x;
            tm->gy = y;
            tm->gtheta = theta;
        }
    }
}

void trajmngr_mix_pos(trajectory_manager_t* tm, const sGenericPosStatus *pos) {
    // TODO use date and position history to retro-correct position

    if(pos->id == ELT_PRIMARY && pos->prop_status.rid > tm->curr_rid) {
        int x, y, theta;
        int x_var, y_var, xy_var, theta_var;
        s2DPUncert_covar cov;
        s2DPUncert_icovar icov_ext, icov_int, icov;

        tm->curr_rid++;

        // get current status
        posctlr_get_pos(&tm->ctlr, &x, &y, &theta);
        posctlr_get_pos_u(&tm->ctlr, &x_var, &y_var, &xy_var, &theta_var);
        cov.x = I2Ds(x);
        cov.y = I2Ds(y);
        cov.theta = (double) theta / dASHIFT;
        cov.a = I2D(I2D((double) x_var / dVarPosSHIFT));
        cov.b = I2D(I2D((double) xy_var / dVarPosSHIFT));
        cov.c = I2D(I2D((double) y_var / dVarPosSHIFT));
        cov.d = (double) theta_var / dRadSHIFT / dRadSHIFT / dVarPosSHIFT;

        // mix position
        covar2icovar(&cov, &icov_int);
        gstatus2icovar(pos, &icov_ext);
        icovar_mix(&icov_ext, &icov_int, &icov);
        icovar2covar(&icov, &cov);

        // set updated position
        x = isD2I(cov.x); // (I << SHIFT)
        y = isD2I(cov.y); // (I << SHIFT)
        theta = iROUND(dASHIFT*cov.theta); // (rad << (RAD_SHIFT + SHIFT))
        x_var = iROUND(D2I(D2I(cov.a))*dVarPosSHIFT);
        y_var = iROUND(D2I(D2I(cov.c))*dVarPosSHIFT);
        xy_var = iROUND(D2I(D2I(cov.b))*dVarPosSHIFT);
        theta_var = iROUND(cov.d * dRadSHIFT * dRadSHIFT * dVarPosSHIFT);

        posctlr_set_pos(&tm->ctlr, x, y, theta);
        posctlr_set_pos_u(&tm->ctlr, x_var, y_var, xy_var, theta_var);
    }
}

void trajmngr_get_pos_status(trajectory_manager_t* tm, sGenericPosStatus *ps) {
    ps->id = ELT_PRIMARY;
    ps->date = bn_intp_micros2s(micros()); // now

    // get position with uncertainty
    s2DPUncert_covar pos;

    int x, y, theta;
    posctlr_get_pos(&tm->ctlr, &x, &y, &theta);
    pos.x = I2Ds(x);
    pos.y = I2Ds(y);
    pos.theta = (double) theta / dASHIFT;

    int x_var, y_var, xy_var, theta_var;
    posctlr_get_pos_u(&tm->ctlr, &x_var, &y_var, &xy_var, &theta_var);
    pos.a = I2D(I2D((double) x_var / dVarPosSHIFT));
    pos.b = I2D(I2D((double) xy_var / dVarPosSHIFT));
    pos.c = I2D(I2D((double) y_var / dVarPosSHIFT));
    pos.d = (double) theta_var / dRadSHIFT / dRadSHIFT / dVarPosSHIFT;

    covar2gstatus(&pos, ps);

    // get trajectory state
    switch(tm->state){
    default:
    case TM_STATE_WAIT_TRAJ:
        ps->prop_status.status = PROP_IDLE;
        break;
    case TM_STATE_WAIT_START:
    case TM_STATE_FOLLOWING:
        ps->prop_status.status = PROP_RUNNING;

#ifdef ARCH_X86_LINUX
        assert(tm->slots[tm->curr_element >> 1].state != SLOT_EMPTY);
#endif

        ps->prop_status.tid = tm->curr_tid;
        ps->prop_status.sid = tm->slots[tm->curr_element >> 1].sid >> 1;
        ps->prop_status.ssid = tm->slots[tm->curr_element >> 1].sid & 1;
        ps->prop_status.sssid = tm->curr_element & 1;
        break;
    }

    // current recalibration id
    ps->prop_status.rid = tm->curr_rid;

    // get speed
    int vx, vy, oz;
    posctlr_get_spd(&tm->ctlr, &vx, &vy, &oz);
    ps->prop_status.spd.vx = IpP2DpSs(vx);
    ps->prop_status.spd.vy = IpP2DpSs(vy);
    ps->prop_status.spd.oz = oz / dASHIFT;
}
