/*
 * pos_history-test.c
 *
 *  Created on: 29 sept. 2014
 *      Author: ludo6431
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <assert.h>

#include "pos_history.h"

//#define OUT
#define NB_STEPS (50)
#define PH_MEMORY_PERIOD (500) // (ms)

int main(int argc, char *argv[]){
    sPeriod p = TP_Ms_CTOR(20); // 20 ms period
    sDate start = TD_LoUs_CTOR(UINT32_MAX - 700500); // start defined so that the date overflows
    const sPeriod ph_MEM_PERIOD = TP_Ms_CTOR(PH_MEMORY_PERIOD);

    sDate d, startMem, endMem;
    sGenericStatus s;
    int i;

    srand(0); // deterministic

#ifdef OUT
    FILE *out = fopen("out.csv", "wb+");
    if(!out){
        exit(1);
    }
#endif

    // generate linear trajectory
    for(d = start, i = 0; i < NB_STEPS; d = tD_addPeriod(d, p), i++){
        printf("d:%.3fs\n", (float)TD_GET_LoUs(d)/1000000.);

        s.date = TD_GET_GlUs(d);
        s.id = ELT_PRIMARY;
        s.prop_status.pos.frame = FRAME_PLAYGROUND;
        s.prop_status.pos.theta = M_PI*i/10.;
        s.prop_status.pos.x = 40. /* cm/s */ * 0.02 /* s/step */ * i /* steps */;
        s.prop_status.pos.y = -20. * 0.02 * i;
        s.prop_status.tid = 0;
        s.prop_status.sid = 0;
        s.prop_status.ssid = 0;
        s.prop_status.speed = 45;

        printf("\tx:%.2fcm, y:%.2fcm, theta:%.2f°\n", s.prop_status.pos.x, s.prop_status.pos.y, s.prop_status.pos.theta * 180. / M_PI);

#ifdef OUT
        fprintf(out, "%u;%.1f;%.1f;%.1f\n", TD_GET_LoUs(d), s.prop_status.pos.x, s.prop_status.pos.y, s.prop_status.pos.theta * 180. / M_PI);
#endif

        ph_enqueue(&s, d);

        if(i == NB_STEPS - 1){
            endMem = d;
        }
    }

    // try to query position in the past
    d = tD_addUs(start, 3000); // force desynchro

    startMem = tD_subPeriod(endMem, ph_MEM_PERIOD); // we only store a position between start and endMem

    for(i = 0; i < NB_STEPS; d = tD_addPeriod(tD_addUs(d, (rand()&1023)-512), p), i++){
        printf("d:%.3fs\n", (float)TD_GET_LoUs(d)/1000000.);

        if(ph_get_pos(&s, d)){
            printf("\tcan't find position!\n");

            // assert out of range of memory
            assert(TD_DIFF_Us(d, start) < 0 || TD_DIFF_Us(d, endMem) > 0);

            continue;
        }

        // assert inside range of memory
        assert(TD_DIFF_Us(d, start) >= 0 && TD_DIFF_Us(d, endMem) <= 0);

#ifdef OUT
        fprintf(out, "%u;%.1f;%.1f;%.1f\n", TD_GET_LoUs(d), s.prop_status.pos.x, s.prop_status.pos.y, s.prop_status.pos.theta * 180. / M_PI);
#endif

        printf("\tx:%.2fcm, y:%.2fcm, theta:%.2f°\n", s.prop_status.pos.x, s.prop_status.pos.y, s.prop_status.pos.theta * 180. / M_PI);
    }

#ifdef OUT
    fclose(out);
#endif

    return 0;
}
