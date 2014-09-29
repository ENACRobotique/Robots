/*
 * pos_history-test.c
 *
 *  Created on: 29 sept. 2014
 *      Author: ludo6431
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "pos_history.h"

//#define OUT

int main(int argc, char *argv[]){
    sPeriod p = TP_Ms_CTOR(20); // 20 ms period
    sDate start = TD_LoUs_CTOR(UINT32_MAX - 700500);//tD_newNow_Lo();

    sDate d;
    sGenericStatus s;
    int i;

    srand(0); // deterministic

#ifdef OUT
    FILE *out = fopen("out.csv", "wb+");
    if(!out){
        exit(1);
    }
#endif

    for(d = start, i = 0; i < 50; d = tD_addPeriod(d, p), i++){
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
        fprintf(out, "%u;%.1f;%.1f;%.1f\n", s.date, s.prop_status.pos.x, s.prop_status.pos.y, s.prop_status.pos.theta * 180. / M_PI);
#endif

        ph_enqueue(&s, d);
    }

    d = tD_addUs(start, 3000);

    for(i = 0; i < 50; d = tD_addPeriod(tD_addUs(d, (rand()&1023)-512), p), i++){
        printf("d:%.3fs\n", (float)TD_GET_LoUs(d)/1000000.);

        if(ph_get_pos(&s, d)){
            printf("\tcan't find position!\n");
            continue;
        }

#ifdef OUT
        fprintf(out, "%u;%.1f;%.1f;%.1f\n", s.date, s.prop_status.pos.x, s.prop_status.pos.y, s.prop_status.pos.theta * 180. / M_PI);
#endif

        printf("\tx:%.2fcm, y:%.2fcm, theta:%.2f°\n", s.prop_status.pos.x, s.prop_status.pos.y, s.prop_status.pos.theta * 180. / M_PI);
    }

#ifdef OUT
    fclose(out);
#endif

    return 0;
}
