#ifdef ARCH_X86_LINUX
#include <assert.h>
#endif

#include "params.h"

#include "trajectory_slot.h"

#define dCAST(v) ((double)(v))
#define dCSHIFT(s) ((double)(1 << (s)))
#define D2Isi(d, i) (D2I(dCAST(d) / dCSHIFT(i)))
#define isD2Isi(d, i) isROUND(D2Isi(d, i))
#define isDpS2IpPs5(dps) isROUND(DpS2IpP(dCAST(dps) / dCSHIFT(5)))
#define _SPDCALC(d, t) (int32_t)((t)>0 ? ((int64_t)(d) * (int64_t)USpP / (int64_t)(t)) : 0)
void _convertMsg2Slot(const sTrajOrientElRaw_t* m, sTrajSlot_t* s, int8_t ssid) {
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

void trajslot_update_with_next(sTrajSlot_t* curr, const sTrajSlot_t* next) {
    if(
            curr->state == SLOT_WAITING_NEXT &&
            next->state != SLOT_EMPTY &&
            next->tid == curr->tid &&
            next->sid == ((curr->sid + 1)&31)
    ) {
        int32_t dt_us = next->seg_start_date - curr->arc_start_date; // duration for arc item in microseconds

#ifdef ARCH_X86_LINUX
        assert(dt_us >= 0);
#endif

        curr->arc_spd = _SPDCALC(curr->arc_len, dt_us);

        curr->state = SLOT_OK;
    }
}

int trajslot_create_from_msg(const sTrajOrientElRaw_t* m, sTrajSlot_t* s1, sTrajSlot_t* s2) {
    int ret = 1;

    _convertMsg2Slot(m, s1, 0);

    if(!m->elts[0].is_last_element) {
        ret = 2;

        _convertMsg2Slot(m, s2, 1);

        trajslot_update_with_next(s1, s2);
    }

    return ret;
}
