/*
 * main.c
 *
 *  Created on: 10 oct. 2013
 *      Author: Ludo6431
 */
#include <signal.h>
#include <stdio.h>
#include <ctype.h>
#include <sys/time.h>
#include <unistd.h>
#include <malloc.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <getopt.h> // parameters parsing
#include <errno.h>

#include "millis.h"

#include "tools.h"
#include "a_star.h"
#include "math_types.h"
#include "math_ops.h"

#include "../botNet/shared/botNet_core.h"
#include "../botNet/shared/bn_debug.h"
#include "../botNet/shared/bn_utils.h"
#include "../../global_errors.h"
#include "node_cfg.h"

#ifdef CTRLC_MENU
static int menu = 0;

void intHandler(int dummy) {
    menu = 1;
}
#endif

void usage(char *cl){
    printf("main ia\n");
    printf("Usage:\n\t%s [options]\n", cl);
    printf("Options:\n");
    printf("\t--log-file, -f        output log file of received messages (overwritten)\n");
    printf("\t--verbose, -v         increases verbosity\n");
    printf("\t--quiet, -q           not verbose\n");
    printf("\t--help, -h, -?        prints this help\n");
}

int main(int argc, char **argv){
    int ret;
    char verbose=1;
    sMsg msgIn, msgOut;
    FILE *fd = NULL;
    sPath_t new_path = {.path = NULL}, curr_path = {.path = NULL};
    int curr_tid = 0;
    sPt_t last_pos = {0., 0.};
    float last_theta = 0.;
    int i;
    // obss send
    uint8_t send_obss_reset = 0, send_obss_idx = 0;
    unsigned int prevSendObss = 0;

#ifdef CTRLC_MENU
    char cmd;
    bn_Address destAd;
    int quit=0,quitMenu=0;
#endif

    // arguments parsing
    while(1){
        static struct option long_options[] = {
                {"log-file",        required_argument,  NULL, 'f'},
                {"verbose",         no_argument,        NULL, 'v'},
                {"quiet",           no_argument,        NULL, 'q'},
                {"help",            no_argument,        NULL, 'h'},
                {NULL,              0,                  NULL, 0}
        };

        int c = getopt_long(argc, argv, "f:vqh?", long_options, NULL);
        if(c == -1)
            break;
        switch(c){
        case 'f':
            if(fd){
                fclose(fd);
            }
            fd = fopen(optarg, "wb+");
            break;
        case 'v':
            verbose++;
            break;
        case 'q':
            verbose = 0;
            break;

        default:
           printf("?? getopt returned character code 0%o ??\n", c);
           /* no break */
        case 'h':
        case '?':
            usage(argv[0]);
            exit(EXIT_FAILURE);
            break;
        }
    }

    bn_attach(E_ROLE_SETUP, role_setup);

    ret = bn_init();
    if (ret < 0){
        printf("bn_init() error #%i\n", -ret);
        exit(1);
    }

#ifdef CTRLC_MENU
    signal(SIGINT, intHandler);
    printf("listening, CTRL+C  for menu\n");
#else
    printf("listening...\n");
#endif

    //main loop
#ifdef CTRLC_MENU
    while (!quit)
#else
    while(1)
#endif
    {
#ifdef CTRLC_MENU
        int nbTraces,f; //for traceroute display
#endif
        usleep(500);

        //receives messages, displays string if message is a debug message
        ret = bn_receive(&msgIn);
        if (ret > 0){
            printf("\x1b[K\x1b[s");

            ret = role_relay(&msgIn);
            if(ret < 0){
                printf("role_relay() error #%i\n", -ret);
            }

            if (verbose>=1) {
                if(msgIn.header.type != E_POS)
                printf("message received from %s (%03hx), type : %s (%hhu)  ", role_string(role_get_role(msgIn.header.srcAddr)), msgIn.header.srcAddr, eType2str(msgIn.header.type), msgIn.header.type);
                if(fd) fprintf(fd,"message received from %hx, type : %s (%hhu)  ",msgIn.header.srcAddr,eType2str(msgIn.header.type),msgIn.header.type);
            }
            switch (msgIn.header.type){
            case E_DEBUG :
                printf("%s",msgIn.payload.debug);
                if(fd) fprintf(fd,"%s",msgIn.payload.debug);
                break;
            case E_POS :
//                printf("robot%hhu@(%fcm,%fcm,%f°)\n", msgIn.payload.pos.id, msgIn.payload.pos.x, msgIn.payload.pos.y, msgIn.payload.pos.theta*180./M_PI);
//                printf("\n");
                if(fd) fprintf(fd,"message received from %hx, type : %s (%hhu)  ",msgIn.header.srcAddr,eType2str(msgIn.header.type),msgIn.header.type);

                // updating position...
                last_pos.x = msgIn.payload.pos.x;
                last_pos.y = msgIn.payload.pos.y;
                last_theta = msgIn.payload.pos.theta;

                if(curr_path.path && msgIn.payload.pos.tid == curr_tid){
                    if(msgIn.payload.pos.ssid){ // circle portion
                        sPt_t *o_c = NULL;
                        sVec_t v;
                        sNum_t r, n, d;

                        o_c = &curr_path.path[msgIn.payload.pos.sid].obs.c;
                        r = curr_path.path[msgIn.payload.pos.sid].obs.r;

                        convPts2Vec(o_c, &last_pos, &v);
                        normVec(&v, &n);

                        d = n - fabs(r);

                        if(fabs(d) > 2.){
                            printf("!!! far from the circle (%.2fcm)...\n", d);
                        }

                        obs[0].moved = 1;
                        obs[0].c.x = o_c->x + v.x*(fabs(curr_path.path[msgIn.payload.pos.sid].obs.r) + 0.1)/n;
                        obs[0].c.y = o_c->y + v.y*(fabs(curr_path.path[msgIn.payload.pos.sid].obs.r) + 0.1)/n;
                        obs[0].r = 0.;
                    }
                    else{ // line portion
                        sNum_t d;
                        sPt_t h;
                        sSeg_t s;

                        convPts2Seg(&curr_path.path[msgIn.payload.pos.sid].p1, &curr_path.path[msgIn.payload.pos.sid].p2, &s);

                        sqdistPt2Seg(&last_pos, &s, &d, &h);

                        if(d > 2.*2.){
                            printf("!!! far from the line (%.2fcm)...\n", sqrt(d));
                        }

                        obs[0].moved = 1;
                        obs[0].c = h;
                        obs[0].r = 0.;
                    }
                }
                else{
                    obs[0].moved = 1;
                    obs[0].c = last_pos;
                    obs[0].r = 0.;
                }
                break;
            case E_GOAL :
                printf("robot%hhu@(%fcm,%fcm,%f°)\n", msgIn.payload.pos.id, msgIn.payload.pos.x, msgIn.payload.pos.y, msgIn.payload.pos.theta*180./M_PI);
                if(fd) fprintf(fd,"message received from %hx, type : %s (%hhu)  ",msgIn.header.srcAddr,eType2str(msgIn.header.type),msgIn.header.type);

                obs[N - 1].moved = 1;
                obs[N - 1].c.x = msgIn.payload.pos.x;
                obs[N - 1].c.y = msgIn.payload.pos.y;
                obs[N - 1].r = 0.;

                break;
            case E_OBS_CFG:
                send_obss_reset = 1;
                send_obss_idx = 0;
                prevSendObss = millis();

                msgOut.header.destAddr = role_get_addr(ROLE_MONITORING);
                msgOut.header.type = E_OBS_CFG;
                msgOut.header.size = sizeof(msgOut.payload.obsCfg);

                msgOut.payload.obsCfg.nb_obs = N;
                msgOut.payload.obsCfg.r_robot = R_ROBOT;

                ret = bn_send(&msgOut);
                if(ret < 0){
                    printf("bn_send(E_OBS_CFG) error #%i\n", -ret);
                }
                break;
            default :
                printf("\n");
                if(fd) fprintf(fd, "\n");
                break;
            }

            printf("pos %.2fcm, %.2fcm, %.1f°\x1b[u", last_pos.x, last_pos.y, last_theta*180./M_PI);
            fflush(stdout);
        }
        else if (ret < 0){
#ifdef CTRLC_MENU
            if (ret == -ERR_SYSERRNO && errno == EINTR){
                menu=1;
            }
            else
#endif
            {
                fprintf(stderr, "bn_receive() error #%i\n", -ret);
                exit(1);
            }
        }

        if(obs[N - 1].moved){
            fill_tgts_lnk();

            if(DIST(0, N - 1) < 1.){
                continue;
            }

            for(i = 0; i < N; i++){
                obs[i].moved = 0;
            }

            if(new_path.path){
                free(new_path.path);
                memset(&new_path, 0, sizeof(new_path));
            }
            a_star(A(0), A(N-1), &new_path);
            printf("new path from 0a to %ua (%.2fcm, %u steps):\n", N-1, new_path.dist, new_path.path_len);
            if(new_path.path){
                curr_tid++;

                for(i = 0; i < new_path.path_len; i++){
                    printf("  %u: p1 x%f y%f, p2 x%f y%f, obs x%f y%f r%.2f, a_l%f s_l%f\n", i, new_path.path[i].p1.x, new_path.path[i].p1.y, new_path.path[i].p2.x, new_path.path[i].p2.y,new_path.path[i].obs.c.x,new_path.path[i].obs.c.y, new_path.path[i].obs.r,new_path.path[i].arc_len,new_path.path[i].seg_len);

                    msgOut.header.type = E_TRAJ;
                    msgOut.header.size = sizeof(msgOut.payload.traj);

                    msgOut.payload.traj.p1_x = new_path.path[i].p1.x;
                    msgOut.payload.traj.p1_y = new_path.path[i].p1.y;
                    msgOut.payload.traj.p2_x = new_path.path[i].p2.x;
                    msgOut.payload.traj.p2_y = new_path.path[i].p2.y;
                    msgOut.payload.traj.seg_len = new_path.path[i].seg_len;

                    msgOut.payload.traj.c_x = new_path.path[i].obs.c.x;
                    msgOut.payload.traj.c_y = new_path.path[i].obs.c.y;
                    msgOut.payload.traj.c_r = new_path.path[i].obs.r;
                    msgOut.payload.traj.arc_len = new_path.path[i].arc_len;

                    msgOut.payload.traj.sid = i;
                    msgOut.payload.traj.tid = curr_tid;

                    ret = role_send(&msgOut);
                    if(ret < 0){
                        printf("role_send() error #%i\n", -ret);
                    }
                }

                if(curr_path.path){
                    free(curr_path.path);
                    memset(&curr_path, 0, sizeof(curr_path));
                }
                memcpy(&curr_path, &new_path, sizeof(curr_path));
                memset(&new_path, 0, sizeof(curr_path));
            }
        }

        if(send_obss_reset && (millis() - prevSendObss > 100)){
            prevSendObss = millis();

            msgOut.header.destAddr = role_get_addr(ROLE_MONITORING);
            msgOut.header.type = E_OBSS;

            for(i=0; send_obss_idx < N && i < MAX_NB_OBSS_PER_MSG; i++, send_obss_idx++){
                msgOut.payload.obss.obs[i].id = send_obss_idx;
                msgOut.payload.obss.obs[i].active = obs[send_obss_idx].active;
                msgOut.payload.obss.obs[i].moved = obs[send_obss_idx].moved;

                msgOut.payload.obss.obs[i].x = (int16_t)(obs[send_obss_idx].c.x*100.);
                msgOut.payload.obss.obs[i].y = (int16_t)(obs[send_obss_idx].c.y*100.);
                msgOut.payload.obss.obs[i].r = (int16_t)(obs[send_obss_idx].r*100.);
            }
            msgOut.payload.obss.nb_obs = i;
            msgOut.header.size = 2 + (i<<3);

            ret = bn_send(&msgOut);
            if(ret < 0){
                printf("bn_send(E_OBSS) error #%i\n", -ret);
            }

            if(send_obss_idx == N){
                send_obss_reset = 0;
            }
        }

        //menu
#ifdef CTRLC_MENU
        if (menu){
            quitMenu=0;
            while (!quitMenu){
                printf("\ndebug reader menu\n");
                printf("s : send debugger address\n");
                printf("p : ping\n");
                printf("t : traceroute\n");
                printf("i : info about this node\n");
                printf("l : add/remove line return to debug strings if missing\n");
                printf("v : increase verbosity\n");
                printf("V : decrease verbosity\n");
                printf("r : return\n");
                printf("q : quit\n");

                while(isspace(cmd=getchar()));

                switch (cmd){
                case 'd':
                    {
                        sMsg msg;
                        msg.header.type = E_ROLE_SETUP;
                        msg.header.destAddr = ADDRD_MAIN_PROP_SIMU;
                        msg.header.size = 2 + 4;
                        msg.payload.roleSetup.nb_steps = 1;
                        // step #0
                        msg.payload.roleSetup.steps[0].step = UPDATE_ACTIONS;
                        msg.payload.roleSetup.steps[0].type = E_DEBUG;
                        msg.payload.roleSetup.steps[0].actions.sendTo.first = ROLE_DEBUG;
                        msg.payload.roleSetup.steps[0].actions.sendTo.second = ROLE_MONITORING;
                        msg.payload.roleSetup.steps[0].actions.relayTo.n1 = 0;
                        msg.payload.roleSetup.steps[0].actions.relayTo.n2 = 0;

                        ret = bn_send(&msg);
                        if(ret < 0){
                            printf("bn_send(E_ROLE_SETUP) error #%i\n", -ret);
                        }
                    }
                    break;
                case 's' :  //sends debug address to distant node
                    do{
                        printf("enter destination address\n");
                        ret = scanf("%hx",&destAd);
                        if (ret != 1){
                            printf("error getting destination address\n");
                        }
                    }while(ret != 1);
                    if ( (ret=bn_debugSendAddr(destAd)) > 0){
                        printf("signalling send\n");
                        quitMenu=1;
                    }
                    else {
                        printf("error while sending : %d\n", ret);

                    }
                    break;
                case 'p' :
                    do{
                         printf("enter destination address\n");
                         ret = scanf("%hx",&destAd);
                         if (ret != 1){
                             printf("error getting destination address\n");
                         }
                     }while(ret != 1);
                     printf("ping %hx : %d ms\n",destAd,bn_ping(destAd));
                    break;
                case 't' :
                    {
                        sTraceInfo *trInfo=NULL;
                        int depth;
                        do{
                             printf("enter destination address\n");
                             ret = scanf("%hx",&destAd);
                             if (ret != 1){
                                 printf("error getting destination address\n");
                             }
                        }while(ret != 1);
                        do{
                             printf("enter depth\n");
                             ret = scanf("%i",&depth);
                             if (ret != 1 || depth <= 0){
                                 printf("error getting depth\n");
                             }
                        }while(ret != 1 || depth <= 0);
                        trInfo = (sTraceInfo *)malloc(depth * sizeof(sTraceInfo));
                        nbTraces=bn_traceroute(destAd,trInfo,depth,1000);
                        for (f=0;f<nbTraces;f++){
                            printf("%hx in %d ms\n",trInfo[f].addr,trInfo[f].ping);
                        }
                    }
                    break;
                case 'i' :  //displays info about current node
#if MYADDRX
                    printf("xBee address:\n");
                    printf("  total : %4hx\n",MYADDRX);
                    printf("  local : %4hx\n",MYADDRX&DEVICEX_MASK);
                    printf("  subnet: %4hx\n",MYADDRX&SUBNET_MASK);
#endif
#if MYADDRI
                    printf("I²C address:\n");
                    printf("  total : %4hx\n",MYADDRI);
                    printf("  local : %4hx\n",MYADDRI&DEVICEI_MASK);
                    printf("  subnet: %4hx\n",MYADDRI&SUBNET_MASK);
#endif
#if MYADDRU
                    printf("UART address:\n");
                    printf("  total : %4hx\n",MYADDRU);
                    printf("  local : %4hx\n",MYADDRU&DEVICEU_MASK);
                    printf("  subnet: %4hx\n",MYADDRU&SUBNET_MASK);
#endif
#if MYADDRD
                    printf("UDP address:\n");
                    printf("  total : %4hx\n",MYADDRD);
                    printf("  local : %4hx\n",MYADDRD&DEVICED_MASK);
                    printf("  subnet: %4hx\n",MYADDRD&SUBNET_MASK);
#endif

                    printf("\n");
                    break;
                case 'l' :
                    oLF^=1;
                    if (oLF) printf("new line will be added if missing");
                    else printf("new line won't be added if missing");
                    break;
                case 'c' :
                    verbose^=1;
                    if (verbose) printf("message info won't be displayed");
                    else printf("message info will be displayed");
                    break;
                case 'r' : quitMenu=1; printf("back to listening, CTRL+C for menu\n\n"); break;
                case 'q' : quitMenu=1; quit=1; break;
                default : break;
                }
            }

            menu=0;
        }
#endif
    }

    if (fd) fclose(fd);
    printf("bye\n");

    return 0;
}
