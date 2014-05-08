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

#include "obj.h"
#include "obj_types.h"
#include "obj_statuses.h"
#include "obj_positions.h"
#include "obj_fire.h"


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
    printf("\t--mode,     -m        AI mode (slave | auto)\n");
    printf("\t--log-file, -f        output log file of received messages (overwritten)\n");
    printf("\t--verbose,  -v        increases verbosity\n");
    printf("\t--quiet,    -q        not verbose\n");
    printf("\t--help,     -h, -?    prints this help\n");
}

void posUpdated(sGenericStatus *status){
    if(status->id != ELT_PRIMARY){
        obs[status->id].active = 1;
        obs[status->id].moved = 1;
        obs[status->id].c.x = status->pos.x;
        obs[status->id].c.y = status->pos.y;

        obs_updated[status->id]++;
    }
}

int main(int argc, char **argv){
    int ret;
    char verbose=1;
    sMsg msgIn, msgOut = {{0}};
    FILE *fd = NULL;
    sPt_t last_pos = {0., 0.};
    float last_theta = 0.;
    float last_speed = 0.;
    unsigned int prevPos = 0;
    int last_tid = 0;
    int i;
    enum{
        E_AI_SLAVE,
        E_AI_AUTO
    } eAIState = E_AI_SLAVE;
    // traj mgmt
    sPath_t new_path = {.path = NULL}, curr_path = {.path = NULL};
    int curr_traj_extract_sid = -1;
    unsigned int prevSendTraj = 0;
    // obss send
    uint8_t send_obss_reset = 0, send_obss_idx = 0;
    unsigned int prevSendObss = 0, prevSendObs = 0;


 /*   //test a supprimer
    sLin_t l={2., 1., -1000., 0};
    sPt_t c={2.,2.};
    sNum_t r=2.;
    sPt_t pt1;
    sPt_t pt2;

    int df=interC2D(&l, &c, r, &pt1, &pt2);
    printf("%d\n",df);
    printf("pt1.x =%f\n", pt1.x);
    printf("pt1.y =%f\n", pt1.y);
    printf("pt2.x =%f\n", pt2.x);
    printf("pt2.y =%f\n", pt2.y);
    getchar();
*/




#ifdef CTRLC_MENU
    char cmd;
    bn_Address destAd;
    int quit=0,quitMenu=0;
#endif

    // arguments parsing
    while(1){
        static struct option long_options[] = {
                {"mode",            required_argument,  NULL, 'm'},
                {"log-file",        required_argument,  NULL, 'f'},
                {"verbose",         no_argument,        NULL, 'v'},
                {"quiet",           no_argument,        NULL, 'q'},
                {"help",            no_argument,        NULL, 'h'},
                {NULL,              0,                  NULL, 0}
        };

        int c = getopt_long(argc, argv, "m:f:vqh?", long_options, NULL);
        if(c == -1)
            break;
        switch(c){
        case 'm':
            if(!strcasecmp(optarg, "slave")){
                eAIState = E_AI_SLAVE;
            }
            else if(!strcasecmp(optarg, "auto")){
                eAIState = E_AI_AUTO;
            }
            break;
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

    {
        sStatusHandlingConfig cfg;
        cfg.has_position = 1;
        cfg.handlerPG = posUpdated;

        // setConfig(ELT_PRIMARY, &cfg);
        setConfig(ELT_SECONDARY, &cfg);
        setConfig(ELT_ADV_PRIMARY, &cfg);
        setConfig(ELT_ADV_SECONDARY, &cfg);
    }

    switch(eAIState){
    case E_AI_AUTO:
        ret = obj_init();
        if(ret < 0){
            printf("obj_init() error #%i\n", -ret);
        }
        break;
    case E_AI_SLAVE:
        obs[0].c.x = 300. - 15.5;
        obs[0].c.y = 200. - 15.8;

        // sending initial position
        msgOut.header.type = E_POS;
        msgOut.header.size = sizeof(msgOut.payload.pos);

        msgOut.payload.pos.id = 0;
        msgOut.payload.pos.theta = -M_PI/2;
        msgOut.payload.pos.u_a = 0;
        msgOut.payload.pos.u_a_theta = 0;
        msgOut.payload.pos.u_b = 0;
        msgOut.payload.pos.x = obs[0].c.x;
        msgOut.payload.pos.y = obs[0].c.y;
        printf("Sending initial position to robot%i (%.2fcm,%.2fcm,%.2f°).\n", msgOut.payload.pos.id, msgOut.payload.pos.x, msgOut.payload.pos.y, msgOut.payload.pos.theta*180./M_PI);
        ret = role_sendAck(&msgOut);
        if(ret <= 0){
            printf("bn_sendAck(E_POS) error #%i\n", -ret);
        }
        break;
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
                {
                	sPt_t new_pos = {.x = msgIn.payload.pos.x, .y = msgIn.payload.pos.y};
                	sNum_t d;

                	if(prevPos){
                		distPt2Pt(&last_pos, &new_pos, &d);

                		last_speed = d*1000./(millis() - prevPos);
                		speed=last_speed;

                		//printf("%.2fcm/s\n", last_speed);
                	}

					prevPos = millis();
                	last_pos = new_pos;
                }

                last_pos.x = msgIn.payload.pos.x;
                last_pos.y = msgIn.payload.pos.y;
                last_theta = msgIn.payload.pos.theta;
                theta_robot = last_theta;
                if(msgIn.payload.pos.tid > last_tid){
                    last_tid = msgIn.payload.pos.tid;
                }

                {
                    sGenericStatus status;

                    status.date = micros(); // XXX
                    status.id = ELT_PRIMARY;
                    status.prop_status.pos.frame = FRAME_PLAYGROUND;
                    status.prop_status.pos.theta = msgIn.payload.pos.theta;
                    status.prop_status.pos_u.a_std = 0.;
                    status.prop_status.pos_u.b_std = 0.;
                    status.prop_status.pos_u.a_angle = 0.;
                    status.prop_status.pos_u.theta = 0.;
                    status.prop_status.pos.x = msgIn.payload.pos.x;
                    status.prop_status.pos.y = msgIn.payload.pos.y;

                    received_new_status(&status);
                }

                if(curr_path.path && msgIn.payload.pos.tid == curr_path.tid){
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
                        obs[0].c.x = o_c->x + v.x*(fabs(r) + 0.1)/n;
                        obs[0].c.y = o_c->y + v.y*(fabs(r) + 0.1)/n;
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
                printf("robot%hhu@(%.2fcm,%.2fcm,%.2f°)\n", msgIn.payload.pos.id, msgIn.payload.pos.x, msgIn.payload.pos.y, msgIn.payload.pos.theta*180./M_PI);
                if(fd) fprintf(fd,"message received from %hx, type : %s (%hhu)  ",msgIn.header.srcAddr,eType2str(msgIn.header.type),msgIn.header.type);

                switch(eAIState){
                case E_AI_SLAVE:
                    obs[N - 1].moved = 1;
                    obs[N - 1].c.x = msgIn.payload.pos.x;
                    obs[N - 1].c.y = msgIn.payload.pos.y;
                    obs[N - 1].r = 0.;
                    obs_updated[N - 1]++;
                    break;
                default:
                    break;
                }

                break;
            case E_OBS_CFG:
                send_obss_reset = 1;
                send_obss_idx = 0;
                prevSendObss = millis();

                for(i = 0; i < N; i++){
                    obs_updated[i] = 1;
                }

                msgOut.header.destAddr = role_get_addr(ROLE_MONITORING);
                msgOut.header.type = E_OBS_CFG;
                msgOut.header.size = sizeof(msgOut.payload.obsCfg);

                msgOut.payload.obsCfg.nb_obs = N;
                msgOut.payload.obsCfg.r_robot = R_ROBOT;
                msgOut.payload.obsCfg.x_min = X_MIN;
                msgOut.payload.obsCfg.x_max = X_MAX;
                msgOut.payload.obsCfg.y_min = Y_MIN;
                msgOut.payload.obsCfg.y_max = Y_MAX;

                ret = bn_send(&msgOut);
                if(ret < 0){
                    printf("bn_send(E_OBS_CFG) error #%i\n", -ret);
                }
                break;
            case E_GENERIC_STATUS:
                received_new_status(&msgIn.payload.genericStatus);
                break;
            case E_IHM_STATUS:
                for(i = 0 ; i < (int)msgIn.payload.ihmStatus.nb_states ; i++){
                    switch(msgIn.payload.ihmStatus.states[i].id){
                        case IHM_STARTING_CORD:
                            starting_cord = msgIn.payload.ihmStatus.states[i].state;
                            printf("## scord: %i\n", starting_cord);
                            break;
                        case IHM_MODE_SWICTH:
                            mode_switch = msgIn.payload.ihmStatus.states[i].state;
                            printf("## smode: %i\n", mode_switch);
                            break;
                        case IHM_LED:
                            break;
                        default:
                            break;
                    }
                }
                break;
            default :
                printf("\n");
                if(fd) fprintf(fd, "\n");
                break;
            }

            printf("pos %.2fcm, %.2fcm, %.1f°", last_pos.x, last_pos.y, last_theta*180./M_PI);
            printf(", armLeft : ");
            printServoPos(&armLeft);
            printf(", armRight : ");
            printServoPos(&armRight);
            printf("\x1b[u");
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

        switch(eAIState){
        case E_AI_SLAVE:
            if(obs[N - 1].moved){
                fill_tgts_lnk();

                for(i = 0; i < N; i++){
                    obs[i].moved = 0;
                }

                if(DIST(0, N - 1) < 1.){
                    continue;
                }

                a_star(A(0), A(N-1), &new_path);
                if(new_path.path){
                    printf("new path from 0a to %ua (%.2fcm, %u steps):\n", N-1, new_path.dist, new_path.path_len);

                    if(curr_path.path){
                        free(curr_path.path);
                    }
                    memcpy(&curr_path, &new_path, sizeof(curr_path));
                    new_path.path = NULL;

                    curr_traj_extract_sid = 0;
                    curr_path.tid = ++last_tid;
                }
                else{
                    printf("no path from 0a to %ua\n", N-1);
                }
            }
            break;
        case E_AI_AUTO:
            obj_step();
            break;
        default:
            break;
        }

        // sending trajectory, one step at a time
        if(curr_path.path && curr_traj_extract_sid < curr_path.path_len && (!curr_traj_extract_sid || (millis() - prevSendTraj > 20))){
            prevSendTraj = millis();

            printf("traj step: p1 x%.2f y%.2f, p2 x%.2f y%.2f, s_l%.2f; obs x%.2f y%.2f r%.2f, a_l%.2f\n", curr_path.path[curr_traj_extract_sid].p1.x, curr_path.path[curr_traj_extract_sid].p1.y, curr_path.path[curr_traj_extract_sid].p2.x, curr_path.path[curr_traj_extract_sid].p2.y, curr_path.path[curr_traj_extract_sid].seg_len, curr_path.path[curr_traj_extract_sid].obs.c.x, curr_path.path[curr_traj_extract_sid].obs.c.y, curr_path.path[curr_traj_extract_sid].obs.r, curr_path.path[curr_traj_extract_sid].arc_len);

            msgOut.header.type = E_TRAJ;
            msgOut.header.size = sizeof(msgOut.payload.traj);

            msgOut.payload.traj.p1_x    = curr_path.path[curr_traj_extract_sid].p1.x;
            msgOut.payload.traj.p1_y    = curr_path.path[curr_traj_extract_sid].p1.y;
            msgOut.payload.traj.p2_x    = curr_path.path[curr_traj_extract_sid].p2.x;
            msgOut.payload.traj.p2_y    = curr_path.path[curr_traj_extract_sid].p2.y;
            msgOut.payload.traj.seg_len = curr_path.path[curr_traj_extract_sid].seg_len;

            msgOut.payload.traj.c_x     = curr_path.path[curr_traj_extract_sid].obs.c.x;
            msgOut.payload.traj.c_y     = curr_path.path[curr_traj_extract_sid].obs.c.y;
            msgOut.payload.traj.c_r     = curr_path.path[curr_traj_extract_sid].obs.r;
            msgOut.payload.traj.arc_len = curr_path.path[curr_traj_extract_sid].arc_len;

            msgOut.payload.traj.sid     = curr_traj_extract_sid++;
            msgOut.payload.traj.tid     = curr_path.tid;

            ret = role_send(&msgOut);
            if(ret < 0){
                printf("role_send(E_TRAJ) error #%i\n", -ret);
            }
        }

        // check if any obs has been updated => synchro with the monitoring interface
        if(!send_obss_reset && (millis() - prevSendObs > 100)){
            prevSendObs = millis();

            for(i = 0; i < N; i++){
                if(obs_updated[i] > 0){
                    send_obss_reset = 1;
                    send_obss_idx = 0;
                    prevSendObss = 0;
                    break;
                }
            }
        }

        // sending obstacles, up to MAX_NB_OBSS_PER_MSG per message
        if(send_obss_reset && (millis() - prevSendObss > 150)){
            prevSendObss = millis();

            msgOut.header.destAddr = role_get_addr(ROLE_MONITORING);
            msgOut.header.type = E_OBSS;

            for(i = 0; send_obss_idx < N && i < MAX_NB_OBSS_PER_MSG; send_obss_idx++){
                if(obs_updated[send_obss_idx] > 0){
                    obs_updated[send_obss_idx] = 0;

                    msgOut.payload.obss.obs[i].id = send_obss_idx;
                    msgOut.payload.obss.obs[i].active = obs[send_obss_idx].active;
                    msgOut.payload.obss.obs[i].moved = obs[send_obss_idx].moved;

                    msgOut.payload.obss.obs[i].x = (int16_t)(obs[send_obss_idx].c.x*100. + 0.5);
                    msgOut.payload.obss.obs[i].y = (int16_t)(obs[send_obss_idx].c.y*100. + 0.5);
                    msgOut.payload.obss.obs[i].r = (int16_t)(obs[send_obss_idx].r*100. + 0.5);

                    i++;
                }
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

//        if(millis() - prevGetPos > 200){
//            sGenericPos *p;
//
//            prevGetPos = millis();
//
//            p = getLastPGPosition(ELT_ADV_PRIMARY);
//            printf("ADV_PRIMARY %p\n", p);
//
//            p = getLastPGPosition(ELT_ADV_SEC);
//            printf("ADV_SEC %p\n", p);
//
//            p = getLastPGPosition(ELT_PRIMARY);
//            printf("PRIMARY %p\n", p);
//
//            p = getLastPGPosition(ELT_SECONDARY);
//            printf("SECONDARY %p\n", p);
//        }

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
