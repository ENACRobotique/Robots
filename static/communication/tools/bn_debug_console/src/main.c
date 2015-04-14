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

#include "../botNet/shared/botNet_core.h"
#include "../network_tools/bn_debug.h"
#include "../network_tools/bn_intp.h"
#include "../network_tools/bn_utils.h"
#include "global_errors.h"
#include "../../core/linux/libraries/Millis/millis.h"
#include "node_cfg.h"


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
    int quit=0,quitMenu=0,err,benchmark=0;
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

    err = bn_init();
    if (err < 0){
        printf("bn_init() error #%i\n", -err);
        exit(1);
    }

    signal(SIGINT, intHandler);
    printf("listening, CTRL+C  for menu\n");

    //main loop
    while (!quit){
        int nbTraces,f; //for traceroute display

        usleep(500);

        //receives messages, displays string if message is a debug message
        err = bn_receive(&msgIn);
        if (err > 0){
            err = role_relay(&msgIn);
            if(err < 0){
                printf("role_relay() error #%i\n", -err);
            }

            if (verbose>=1) {
                printf("message received from %s (%03hx), type : %s (%hhu), seq : %02hhu ", role_string(role_get_role(msgIn.header.srcAddr)), msgIn.header.srcAddr, eType2str(msgIn.header.type), msgIn.header.type, msgIn.header.seqNum);
                if(fd) fprintf(fd,"message received from %hx, type : %s (%hhu), seq : %02hhu ",msgIn.header.srcAddr,eType2str(msgIn.header.type),msgIn.header.type, msgIn.header.seqNum);
            }
            switch (msgIn.header.type){
            case E_GENERIC_STATUS:
                printf("%.2fcm, %.2fcm, %.2f°\n", msgIn.payload.genericStatus.prop_status.pos.x, msgIn.payload.genericStatus.prop_status.pos.y, msgIn.payload.genericStatus.prop_status.pos.theta*180./M_PI);
                break;
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
            case E_POS :
                printf("robot%hhu@(%fcm,%fcm,%f°)\n", msgIn.payload.pos.id, msgIn.payload.pos.x, msgIn.payload.pos.y, msgIn.payload.pos.theta*180./M_PI);
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
        else if (err < 0){
            if (err == -ERR_SYSERRNO && errno == EINTR){
                sigint=1;
            }
            else{
                fprintf(stderr, "bn_receive() error #%i\n", -err);
                if(err == -ERR_SYSERRNO){
                    fprintf(stderr, "errno=%i\n", errno);
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

                err = !fgets(input, sizeof(input), stdin);
                p = &input[0];
                while(*p && isspace(*p)) p++;

                switch (*p){
                case 'e':{
                    sMsg msg = {{0}};
                    static int angle = 50;
                    angle = 180 - angle;

                    msg.header.destAddr = ADDRU1_MAIN_IO;
                    msg.header.type = E_SERVOS;
                    msg.payload.servos.nb_servos = 3;
                    msg.header.size = 2 + msg.payload.servos.nb_servos * sizeof(msg.payload.servos.servos[0]);
                    msg.payload.servos.servos[0].id = SERVO_PRIM_CORN1_RAMP;
                    msg.payload.servos.servos[0].angle = angle;
                    msg.payload.servos.servos[1].id = SERVO_PRIM_CORN_DOOR;
                    msg.payload.servos.servos[1].angle = 140 - angle;
                    msg.payload.servos.servos[2].id = SERVO_PRIM_GLASS3_HOLD;
                    msg.payload.servos.servos[2].angle = 30 + angle;

                    bn_send(&msg);
                    break;
                }
                case 'f':{
                    sMsg msg = {{0}};
                    float angle;

                    printf("angle (deg): "); fflush(stdout);
                    err = scanf("%f", &angle);
                    if (err != 1){
                        printf("error getting us setpoint\n");
                    }

                    msg.header.destAddr = ADDRU1_MAIN_IO;
                    msg.header.type = E_SERVOS;
                    msg.header.size = 2 + 1 * sizeof(msg.payload.servos.servos[0]);
                    msg.payload.servos.nb_servos = 1;
                    msg.payload.servos.servos[0].id = SERVO_PRIM_CORN1_RAMP;
                    msg.payload.servos.servos[0].angle = angle;

                    bn_send(&msg);
                    break;
                }
                case 'g':{
                    sMsg msg = {{0}};
                    float angle;
                    int id;

                    printf(" 0:SERVO_PRIM_DOOR\n");
                    printf(" 1:SERVO_PRIM_FIRE1\n");
                    printf(" 2:SERVO_PRIM_FIRE2\n");
                    printf(" 3:SERVO_PRIM_ARM_LEFT\n");
                    printf(" 4:SERVO_PRIM_ARM_RIGHT\n");

                    printf("id: "); fflush(stdout);
                    err = scanf("%i", &id);
                    if (err != 1){
                        printf("error getting servo id\n");
                    }

                    printf("angle (deg): "); fflush(stdout);
                    err = scanf("%f", &angle);
                    if (err != 1){
                        printf("error getting angle setpoint\n");
                    }

                    msg.header.destAddr = ADDRU1_MAIN_IO;
                    msg.header.type = E_SERVOS;
                    msg.header.size = 2 + 1 * sizeof(msg.payload.servos.servos[0]);
                    msg.payload.servos.nb_servos = 1;
                    msg.payload.servos.servos[0].id = id;
                    msg.payload.servos.servos[0].angle = angle;

                    bn_send(&msg);
                    break;
                }
                case 'h':
                    printf("Syncing propulsion..."); fflush(stdout);
                    bn_intp_sync(role_get_addr(ROLE_PROPULSION), 100);
                    printf("done\n");
                    break;
                case 'w':{
                    sMsg msg = {{0}};

                    msg.header.destAddr = role_get_addr(ROLE_PROPULSION);
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

                    msg.header.destAddr = role_get_addr(ROLE_PROPULSION);
                    msg.header.type = E_SPEED_SETPOINT;
                    msg.header.size = sizeof(msg.payload.speedSetPoint);

                    do{
                        printf("speed (cm/s): ");
                        err = !fgets(input, sizeof(input), stdin);
                        err = err?0:sscanf(input, "%f", &msg.payload.speedSetPoint.speed);
                    }while(err != 1 || msg.payload.speedSetPoint.speed < -150. || msg.payload.speedSetPoint.speed > 150.);

                    bn_send(&msg);
                    break;
                }
                case 's' :  //sends debug address to distant node
                    do{
                        printf("enter destination address\n");
                        err = scanf("%hx",&destAd);
                        if (err != 1){
                            printf("error getting destination address\n");
                        }
                    }while(err != 1);
                    while(getchar() != '\n');
                    if ( (err=bn_debugSendAddr(destAd)) > 0){
                        printf("signalling send\n");
                        quitMenu=1;
                    }
                    else {
                        printf("error while sending : %d\n", err);
                    }
                    break;
                case 'p' :
                    do{
                        printf("enter destination address\n");
                        err = scanf("%hx",&destAd);
                        if (err != 1){
                            printf("error getting destination address\n");
                        }
                    }while(err != 1);
                    while(getchar() != '\n');
                    printf("ping %hx : %d ms\n",destAd,bn_ping(destAd));
                    break;
                case 't' :
                    {
                        sTraceInfo *trInfo=NULL;
                        int depth;
                        do{
                             printf("enter destination address\n");
                             err = scanf("%hx",&destAd);
                             if (err != 1){
                                 printf("error getting destination address\n");
                             }
                        }while(err != 1);
                        while(getchar() != '\n');
                        do{
                             printf("enter depth\n");
                             err = scanf("%i",&depth);
                             if (err != 1 || depth <= 0){
                                 printf("error getting depth\n");
                             }
                        }while(err != 1 || depth <= 0);
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
                        err = !fgets(input, sizeof(input), stdin);
                        err = err?0:sscanf(input, "%hx", &bench_dest_addr);
                    }while(err != 1);

                    do{
                        printf("msg size: ");
                        err = !fgets(input, sizeof(input), stdin);
                        err = err?0:sscanf(input, "%i", &bench_sz_msg);
                    }while(err != 1 || bench_sz_msg < 0 || bench_sz_msg > BN_MAX_PDU - sizeof(msgIn.header));

                    do{
                        printf("msg period: ");
                        err = !fgets(input, sizeof(input), stdin);
                        err = err?0:sscanf(input, "%i", &bench_period);
                    }while(err != 1 || bench_period < 2);

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

                err = bn_sendAck(&msgOut);
                if(err <= 0){
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
