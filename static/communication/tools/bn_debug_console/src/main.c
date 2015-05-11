/*
 * main.c
 *
 *  Created on: 10 oct. 2013
 *      Author: quentin
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

#include "botNet_core.h"
#include "bn_debug.h"
#include "bn_intp.h"
#include "bn_utils.h"
#include "global_errors.h"
#include "millis.h"
#include "node_cfg.h"
#include "roles.h"


static int sigint = 0;

void intHandler(int dummy) {
    sigint = 1;
}

void usage(char *cl){
    printf("Botnet console\n");
    printf("Usage:\n\t%s [options]\n", cl);
    printf("Options:\n");
    printf("\t--log-file, -f        output log file of received messages (overwritten)\n");
    printf("\t--add-return, -r      adds missing line return to debug strings (default)\n");
    printf("\t--no-add-return, -R   do not add missing line return to debug strings\n");
    printf("\t--verbose, -v         increases verbosity\n");
    printf("\t--quiet, -q           not verbose\n");
    printf("\t--help, -h, -?        prints this help\n");
}

int main(int argc, char **argv){
    int quit=0,quitMenu=0,ret,benchmark=0;
    char oLF=1,verbose=1;           //booleans used for display options, add new line char if missing (default yes), pure console mode (default no)
    FILE *fd = NULL;

    sMsg msgIn;
    char *p;
    bn_Address destAd;

    // benchmark mode
    bn_Address bench_dest_addr = 0;
    int bench_sz_msg = 0, bench_period = 2, bench_nb_msg_lost = 0, bench_nb_msg_total = 0;
    unsigned int prevBench;
    unsigned int bench_time_ok = 0, bench_time_ko = 0;
    #define BENCH_TIME_SHIFT (3)

    // arguments parsing
    while(1){
        static struct option long_options[] = {
                {"log-file",        required_argument,  NULL, 'f'},
                {"add-return",      no_argument,        NULL, 'r'},
                {"no-add-return",   no_argument,        NULL, 'R'},
                {"verbose",         no_argument,        NULL, 'v'},
                {"quiet",           no_argument,        NULL, 'q'},
                {"help",            no_argument,        NULL, 'h'},
                {NULL,              0,                  NULL, 0}
        };

        int c = getopt_long(argc, argv, "f:rRvqh?", long_options, NULL);
        if(c == -1)
            break;
        switch(c){
        case 'f':
            if(fd){
                fclose(fd);
            }
            fd = fopen(optarg, "wb+");
            break;
        case 'r' :
            oLF=1;
            break;
        case 'R' :
            oLF=0;
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
        printf("bn_init() failed: %s (#%i)\n", getErrorStr(-ret), -ret);
        exit(1);
    }

    signal(SIGINT, intHandler);
    printf("listening, CTRL+C  for menu\n");

    //main loop
    while (!quit){
        int nbTraces,f; //for traceroute display

        usleep(500);

        //receives messages, displays string if message is a debug message
        ret = bn_receive(&msgIn);
        if (ret > 0){
            ret = role_relay(&msgIn);
            if(ret < 0){
                printf("role_relay() failed: %s (#%i)\n", getErrorStr(-ret), -ret);
            }

            if (verbose>=1) {
                printf("message received from %s (%03hx), type : %s (%hhu), seq : %02hhu ", role_string(role_get_role(msgIn.header.srcAddr)), msgIn.header.srcAddr, eType2str(msgIn.header.type), msgIn.header.type, msgIn.header.seqNum);
                if(fd) fprintf(fd,"message received from %hx, type : %s (%hhu), seq : %02hhu ",msgIn.header.srcAddr,eType2str(msgIn.header.type),msgIn.header.type, msgIn.header.seqNum);
            }
            switch (msgIn.header.type){
            case E_ASSERV_STATS :
                {
                    int i;
                    sAsservStats *as = &msgIn.payload.asservStats;
                    printf("seq %hu\n", as->nb_seq);
                    if(fd) fprintf(fd, "seq %hu\n", as->nb_seq);
                    for(i=0; i<NB_ASSERV_STEPS_PER_MSG; i++){
                        printf("~%hu,%hi,%hi,%hi,%hi,%hi,%hi\n", as->steps[i].delta_t, as->steps[i].ticks_l, as->steps[i].consigne_l, as->steps[i].out_l, as->steps[i].ticks_r, as->steps[i].consigne_r, as->steps[i].out_r);
                        if(fd) fprintf(fd, "~%hu,%hi,%hi,%hi,%hi,%hi,%hi\n", as->steps[i].delta_t, as->steps[i].ticks_l, as->steps[i].consigne_l, as->steps[i].out_l, as->steps[i].ticks_r, as->steps[i].consigne_r, as->steps[i].out_r);
                    }
                }
                break;
            case E_POS_STATS :
                {
                    int i;
                    sPosStats *ps = &msgIn.payload.posStats;
                    printf("seq %hu\n", ps->nb_seq);
                    if(fd) fprintf(fd, "seq %hu\n", ps->nb_seq);
                    for(i=0; i<NB_POS_STEPS_PER_MSG; i++){
                        printf("~%hu,%.2f,%.2f,%.2f\n", ps->steps[i].delta_t, ((float)ps->steps[i].x)/4., ((float)ps->steps[i].y)/4., ((float)ps->steps[i].theta)/10.);
//                        if(fd) fprintf(fd, "~%hu,%hi,%hi,%hi,%hi,%hi,%hi\n", ps->steps[i].delta_t, ps->steps[i].ticks_l, ps->steps[i].consigne_l, ps->steps[i].out_l, ps->steps[i].ticks_r, as->steps[i].consigne_r, as->steps[i].out_r);
                    }
                }
                break;
            case E_DEBUG :
                printf("%s",msgIn.payload.debug);
                if(fd) fprintf(fd,"%s",msgIn.payload.debug);
                if (oLF){
                    if (!strlen((char*)msgIn.payload.debug) || (strlen((char*)msgIn.payload.debug) && msgIn.payload.debug[(unsigned int)strlen((char*)msgIn.payload.debug)-1]!='\n')) {
                        printf("\n");
                        if(fd) fprintf(fd,"\n");
                    }
                }
                break;
            case E_GENERIC_POS_STATUS:
                printf("robot%hhu@(%fcm,%fcm,%f°)\n", msgIn.payload.genericPosStatus.id, msgIn.payload.genericPosStatus.pos.x, msgIn.payload.genericPosStatus.pos.y, msgIn.payload.genericPosStatus.pos.theta*180./M_PI);
                if(fd) fprintf(fd,"message received from %hx, type : %s (%hhu)  ",msgIn.header.srcAddr,eType2str(msgIn.header.type),msgIn.header.type);
                break;
            default :
                if (verbose>=1){
                    printf("\n");
                    if (fd) fprintf(fd,"\n");
                }

                break;
            }
        }
        else if (ret < 0){
            if (ret == -ERR_SYSERRNO && errno == EINTR){
                sigint=1;
            }
            else{
                fprintf(stderr, "bn_receive() failed: %s (#%i)\n", getErrorStr(-ret), -ret);
                if(ret == -ERR_SYSERRNO){
                    fprintf(stderr, "errno=%s (#%i)\n", strerror(errno), errno);
                }
                exit(1);
            }
        }

        //menu
        if (sigint){
            quitMenu=0;

            if(benchmark){
                float period, period_ok, period_ko;

                period = (float)bench_period;
                period_ok = (float)bench_time_ok/((float)(1<<BENCH_TIME_SHIFT));
                period_ko = (float)bench_time_ko/((float)(1<<BENCH_TIME_SHIFT));

                if(period_ok > period){
                    period = period_ok;

                    printf("/!\\ Couldn't sustain the desired period /!\\\n");
                }

                benchmark=0;
                quitMenu=1;

                printf("%i/%i (%.2f%%) messages lost\n", bench_nb_msg_lost, bench_nb_msg_total, 100.*bench_nb_msg_lost/bench_nb_msg_total);
                printf("mean ok time: %.2fms\n", period_ok);
                printf("mean ko time: %.2fms\n", period_ko);
                printf("%.2fbauds, %.2fbit/s, %.2fo/s\n",
                        1000. *(bench_sz_msg + 2*sizeof(msgIn.header))*8. *(bench_nb_msg_total-bench_nb_msg_lost)/bench_nb_msg_total /period,
                        1000. *bench_sz_msg*8.                            *(bench_nb_msg_total-bench_nb_msg_lost)/bench_nb_msg_total /period,
                        1000. *bench_sz_msg                               *(bench_nb_msg_total-bench_nb_msg_lost)/bench_nb_msg_total /period
                );
            }

            while (!quitMenu){
                char input[32];

                printf("\ndebug reader menu\n");
                printf("a : INIT: send RoleSetup messages to prop and moni to tell ai is simulated\n");
                printf("h : INIT: synchronize time on propulsion\n");
                printf("f : start RGB Led debug\n");
                printf("g : Start servo calibration\n");
                printf("d : send speed setpoint to the primary robot\n");
                printf("s : send debugger address\n");
                printf("p : ping\n");
                printf("t : traceroute\n");
                printf("b : benchmark mode\n");
                printf("i : info about this node\n");
                printf("l : add/remove line return to debug strings if missing\n");
                printf("v : increase verbosity\n");
                printf("V : decrease verbosity\n");
                printf("r : return\n");
                printf("q : quit\n");

                ret = !fgets(input, sizeof(input), stdin);
                p = &input[0];
                while(*p && isspace(*p)) p++;

                switch (*p){
                case 'a':{ // setup nodes for simulated AI
                    sMsg msg = {{0}};

                    msg.header.type = E_ROLE_SETUP;
                    msg.header.destAddr = role_get_addr(ROLE_PRIM_PROPULSION);
                    msg.payload.roleSetup.nb_steps = 1;
                    msg.header.size = 2 + 4*msg.payload.roleSetup.nb_steps;
                    // step #0
                    msg.payload.roleSetup.steps[0].step_type = UPDATE_ADDRESS;
                    msg.payload.roleSetup.steps[0].role = ROLE_PRIM_AI;
                    msg.payload.roleSetup.steps[0].address = ADDRD1_MAIN_AI_SIMU;

                    printf("Sending RoleSetup message to propulsion... "); fflush(stdout);
                    ret = bn_sendAck(&msg);
                    if(ret < 0){
                        printf("FAILED: %s (#%i)\n", getErrorStr(-ret), -ret);
                    }
                    else{
                        printf("OK!\n");
                    }

                    msg.header.type = E_ROLE_SETUP;
                    msg.header.destAddr = role_get_addr(ROLE_MONITORING);
                    msg.payload.roleSetup.nb_steps = 1;
                    msg.header.size = 2 + 4*msg.payload.roleSetup.nb_steps;
                    // step #0
                    msg.payload.roleSetup.steps[0].step_type = UPDATE_ADDRESS;
                    msg.payload.roleSetup.steps[0].role = ROLE_PRIM_AI;
                    msg.payload.roleSetup.steps[0].address = ADDRD1_MAIN_AI_SIMU;

                    printf("Sending RoleSetup message to monitoring... "); fflush(stdout);
                    ret = bn_sendAck(&msg);
                    if(ret < 0){
                        printf("FAILED: %s (#%i)\n", getErrorStr(-ret), -ret);
                    }
                    else{
                        printf("OK!\n");
                    }

                    msg.header.type = E_ROLE_SETUP;
                    msg.header.destAddr = ADDRI_MAIN_IO;
                    msg.payload.roleSetup.nb_steps = 1;
                    msg.header.size = 2 + 4*msg.payload.roleSetup.nb_steps;
                    // step #0
                    msg.payload.roleSetup.steps[0].step_type = UPDATE_ADDRESS;
                    msg.payload.roleSetup.steps[0].role = ROLE_PRIM_AI;
                    msg.payload.roleSetup.steps[0].address = ADDRD1_MAIN_AI_SIMU;

                    printf("Sending RoleSetup message to ADDRI_MAIN_IO... "); fflush(stdout);
                    ret = bn_sendAck(&msg);
                    if(ret < 0){
                        printf("FAILED: %s (#%i)\n", getErrorStr(-ret), -ret);
                    }
                    else{
                        printf("OK!\n");
                    }


                    break;
                }
                case 'h':{
                    printf("Syncing propulsion... "); fflush(stdout);
                    ret = bn_intp_sync(role_get_addr(ROLE_PRIM_PROPULSION), 100);
                    if(ret < 0){
                        printf("FAILED: %s (#%i)\n", getErrorStr(-ret), -ret);
                    }
                    else{
                        printf("OK!\n");
                    }
                    break;
                }
                case 'v':
                    verbose++;
                    break;
                case 'V':
                    verbose--;
                    verbose=(verbose<0?0:verbose);
                    break;
                case 'f':{
                    sMsg msg = {{0}};
                    sRGB color1, color2;
                    unsigned int time1, time2, nb, red1, red2, green1, green2, blue1, blue2;

                    printf("1st color (red): "); fflush(stdout);
                    ret = scanf("%i", &red1);
                    if (ret != 1){
                        printf("error getting color (red)\n");
                    }

                    printf("1st color (green): "); fflush(stdout);
                    ret = scanf("%i", &green1);
                    if (ret != 1){
                        printf("error getting color (green)\n");
                    }

                    printf("1st color (blue): "); fflush(stdout);
                    ret = scanf("%i", &blue1);
                    if (ret != 1){
                        printf("error getting color (blue)\n");
                    }

                    printf("2nd color (red): "); fflush(stdout);
                    ret = scanf("%i", &red2);
                    if (ret != 1){
                        printf("error getting color (red)\n");
                    }

                    printf("2nd color (green): "); fflush(stdout);
                    ret = scanf("%i", &green2);
                    if (ret != 1){
                        printf("error getting color (green)\n");
                    }

                    printf("2nd color (blue): "); fflush(stdout);
                    ret = scanf("%i", &blue2);
                    if (ret != 1){
                        printf("error getting color (blue)\n");
                    }

                    printf("time displaying 1st color (ms): "); fflush(stdout);
                    ret = scanf("%i", &time1);
                    if (ret != 1){
                        printf("error getting time1\n");
                    }

                    printf("time displaying 2nd color (ms): "); fflush(stdout);
                    ret = scanf("%i", &time2);
                    if (ret != 1){
                        printf("error getting time2\n");
                    }

                    printf("number of repetitions: "); fflush(stdout);
                    ret = scanf("%i", &nb);
                    if (ret != 1){
                        printf("error getting repetitions\n");
                    }

                    color1.red = red1; color1.green = green1; color1.blue = blue1;
                    color2.red = red2; color2.green = green2; color2.blue = blue2;

                    msg.header.destAddr = ADDRI_MAIN_IO;
                    msg.header.type = E_IHM_STATUS;
                    msg.header.size = 2 + 1 * sizeof(msg.payload.ihmStatus.states[0]);
                    msg.payload.ihmStatus.nb_states = 1;
                    msg.payload.ihmStatus.states[0].id = IHM_LED;
                    msg.payload.ihmStatus.states[0].state.color1 = color1;
                    msg.payload.ihmStatus.states[0].state.color2 = color2;
                    msg.payload.ihmStatus.states[0].state.time1 = time1;
                    msg.payload.ihmStatus.states[0].state.time2 = time2;
                    msg.payload.ihmStatus.states[0].state.nb = nb;

                    bn_send(&msg);
                    break;
                }
                case 'g':{
                    sMsg msg = {{0}};
                    float angle;
                    int club_id, hw_id;

                    printf("club id: "); fflush(stdout);
                    ret = scanf("%i", &club_id);
                    if (ret != 1){
                        printf("error getting servo club id\n");
                    }

                    printf("hardware id: "); fflush(stdout);
                    ret = scanf("%i", &hw_id);
                    if (ret != 1){
                        printf("error getting servo hardware id\n");
                    }

                    printf("angle (deg): "); fflush(stdout);
                    ret = scanf("%f", &angle);
                    if (ret != 1){
                        printf("error getting angle setpoint\n");
                    }

                    msg.header.destAddr = ADDRI_MAIN_IO;
                    msg.header.type = E_SERVOS;
                    msg.header.size = 2 + 1 * sizeof(msg.payload.servos.servos[0]);
                    msg.payload.servos.nb_servos = 1;
                    msg.payload.servos.servos[0].club_id = club_id;
                    msg.payload.servos.servos[0].hw_id = hw_id;
                    msg.payload.servos.servos[0].angle = angle;

                    bn_send(&msg);
                    break;
                }
                case 'w':{
                    sMsg msg = {{0}};

                    msg.header.destAddr = role_get_addr(ROLE_PRIM_PROPULSION);
                    msg.header.type = E_POS_QUERY;
                    msg.header.size = sizeof(msg.payload.posQuery);

                    msg.payload.posQuery.date = (int32_t)micros() - 200000;
                    msg.payload.posQuery.id = ELT_PRIMARY;

                    bn_send(&msg);

                    printf("Sent position query\n");
                    break;
                }
                case 'd':{ // sends new setpoint to the propulsion // FIXME: use sGenericStatus
                    sMsg msg = {{0}};

                    msg.header.destAddr = role_get_addr(ROLE_PRIM_PROPULSION);
                    msg.header.type = E_SPEED_SETPOINT;
                    msg.header.size = sizeof(msg.payload.speedSetPoint);

                    do{
                        printf("speed (cm/s): ");
                        ret = !fgets(input, sizeof(input), stdin);
                        ret = ret?0:sscanf(input, "%f", &msg.payload.speedSetPoint.speed);
                    }while(ret != 1 || msg.payload.speedSetPoint.speed < -150. || msg.payload.speedSetPoint.speed > 150.);

                    bn_send(&msg);
                    break;
                }
                case 's' :  //sends debug address to distant node
                    do{
                        printf("enter destination address\n");
                        ret = scanf("%hx",&destAd);
                        if (ret != 1){
                            printf("error getting destination address\n");
                        }
                    }while(ret != 1);
                    while(getchar() != '\n');
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
                    while(getchar() != '\n');

                    ret = bn_ping(destAd);
                    if(ret < 0){
                        printf("bn_ping(%hx) failed: %s (#%i)\n", destAd, getErrorStr(-ret), -ret);
                    }
                    else{
                        printf("ping %hx : %d ms\n",destAd,ret);
                    }
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
                        while(getchar() != '\n');
                        do{
                             printf("enter depth\n");
                             ret = scanf("%i",&depth);
                             if (ret != 1 || depth <= 0){
                                 printf("error getting depth\n");
                             }
                        }while(ret != 1 || depth <= 0);
                        while(getchar() != '\n');
                        trInfo = (sTraceInfo *)malloc(depth * sizeof(sTraceInfo));
                        nbTraces=bn_traceroute(destAd,trInfo,depth,1000);
                        for (f=0;f<nbTraces;f++){
                            printf("%hx in %d ms\n",trInfo[f].addr,trInfo[f].ping);
                        }
                    }
                    break;
                case 'b': // start benchmark mode
                {
                    do{
                        printf("dest address: ");
                        ret = !fgets(input, sizeof(input), stdin);
                        ret = ret?0:sscanf(input, "%hx", &bench_dest_addr);
                    }while(ret != 1);

                    do{
                        printf("msg size: ");
                        ret = !fgets(input, sizeof(input), stdin);
                        ret = ret?0:sscanf(input, "%i", &bench_sz_msg);
                    }while(ret != 1 || bench_sz_msg < 0 || bench_sz_msg > BN_MAX_PDU - sizeof(msgIn.header));

                    do{
                        printf("msg period: ");
                        ret = !fgets(input, sizeof(input), stdin);
                        ret = ret?0:sscanf(input, "%i", &bench_period);
                    }while(ret != 1 || bench_period < 2);

                    bench_time_ok = 0;
                    bench_time_ko = 0;
                    bench_nb_msg_lost = 0;
                    bench_nb_msg_total = 0;
                    benchmark = 1;
                    quitMenu = 1;
                    prevBench = millis();
                    break;
                }
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

            sigint=0;
        }

        if(benchmark){
            if(millis() - prevBench > bench_period){
                sMsg msgOut = {{0}};

                prevBench = millis();

                msgOut.header.destAddr = bench_dest_addr;
                msgOut.header.size = bench_sz_msg;
                msgOut.header.type = E_DATA;

                ret = bn_sendAck(&msgOut);
                if(ret <= 0){
                    if(!bench_nb_msg_lost){
                        bench_time_ko = (millis() - prevBench)<<BENCH_TIME_SHIFT;
                    }
                    else{
                        bench_time_ko = bench_time_ko - (bench_time_ko >> BENCH_TIME_SHIFT) + (millis() - prevBench);
                    }
                    bench_nb_msg_lost++;
                }
                else{
                    if(!(bench_nb_msg_total - bench_nb_msg_lost)){
                        bench_time_ok = (millis() - prevBench)<<BENCH_TIME_SHIFT;
                    }
                    else{
                        bench_time_ok = bench_time_ok - (bench_time_ok >> BENCH_TIME_SHIFT) + (millis() - prevBench);
                    }
                }
                bench_nb_msg_total++;
            }
        }

    }

    if (fd) fclose(fd);
    printf("bye\n");

    return 0;
}
