/*
 * main.c
 *
 *  Created on: 10 oct. 2013
 *      Author: Ludo6431
 *
 * main.cpp
 *
 *  Created on: 21 dec. 2014
 *      Author: Seb
 */

#include <cstdlib>
#include <cstring>
#include <getopt.h> //already exist extern "C"

extern "C"{
#include <unistd.h> //for usleep
#include "roles.h"
#include "global_errors.h"
#include "bn_intp.h"
#include "messages-position.h"
}

#include "botNet_core.h"
#include "communications.h"
#include "tools.h"
#include "net.h"
#include "GeometryTools.h"
#include "init_robots.h"
#include "environment.h"

#define SYNC_PROP_IA    // sync is done by IA, and only between IA and prop
//#define SYNC_TURRET     // sync is done by turret

#ifdef CTRLC_MENU
static int menu = 0;

void intHandler(int dummy) {
    menu = 1;
}
#endif

void exitAI(int status){
    cout << "\033[0m";
    exit(status);
}

void usage(char *cl) {
    printf("main ia\n");
    printf("Usage:\n\t%s [options]\n", cl);
    printf("Options:\n");
    printf("\t--mode,     -m        AI mode (slave | auto | prog)\n");
    printf("\t--log-file, -f        output log file of received messages (overwritten)\n");
    printf("\t--verbose,  -v        increases verbosity\n");
    printf("\t--quiet,    -q        not verbose\n");
    printf("\t--help,     -h, -?    prints this help\n");
}

int main(int argc, char **argv) {
    int ret;
    eAIState_t eAIState = E_AI_SLAVE;
    bool simu_primary = true;
    bool holo_primary = true;
    bool hmi_simu_primary = true;
    eColor_t color_primary = GREEN;


#ifdef CTRLC_MENU
    char cmd;
    bn_Address destAd;
    int quit=0,quitMenu=0;
#endif

    // arguments parsing
    while (1) {
        static struct option long_options[] = { { "mode", required_argument, NULL, 'm' }, { "log-file", required_argument, NULL, 'f' }, { "primary", required_argument, NULL, 'p' }, { "secondary", required_argument, NULL, 's' }, { "adv-primary", required_argument, NULL, 'a' }, { "adv-secondary", required_argument, NULL, 'b' }, { "verbose", no_argument, NULL, 'v' }, { "quiet", no_argument, NULL, 'q' }, { "help", no_argument, NULL, 'h' }, { NULL, 0, NULL, 0 } };

        int c = getopt_long(argc, argv, "m:f:p:s:a:b:vqh?", long_options, NULL);
        if (c == -1)
            break;
        switch (c) {
            case 'm':
                if (!strcasecmp(optarg, "slave")) {
                    eAIState = E_AI_SLAVE;
                }
                else if (!strcasecmp(optarg, "prog")) {
                    eAIState = E_AI_PROG;
                }
                else if (!strcasecmp(optarg, "auto")) {
                    eAIState = E_AI_AUTO;
                }
                break;
            case 'f':
                logs.changeFile(optarg);
                break;
            case 'p':
                if(strstr(optarg, "real")){
                    simu_primary = false;
                    hmi_simu_primary = false;
                }
                else if(strstr(optarg, "simu"))
                    simu_primary = true;
                if(strstr(optarg, "axle"))
                    holo_primary = false;
                else if(strstr(optarg, "holo"))
                    holo_primary = true;
                if(strstr(optarg, "green"))
                    color_primary = GREEN;
                else if(strstr(optarg, "yellow"))
                    color_primary = YELLOW;
                break;
            case 's':
                break;
            case 'a':
                break;
            case 'b':
                break;
            case 'v':
                verbose++;
                break;
            case 'q':
                verbose = 0;
                break;

            default:
                logs << ERR << "?? getopt returned character code 0" << c << "??";
                /* no break */
            case 'h':
            case '?':
                usage(argv[0]);
                exit(EXIT_FAILURE);
                break;
        }
    }

    // network initialization
    while(1){
        if ((ret = bn_attach(E_ROLE_SETUP, role_setup)) < 0){
            logs << ERR << "bn_attach() error : " << getErrorStr(-ret) << "(#" << -ret << ")\n";
            //exitAI(EXIT_FAILURE);
        }
        else
            break;
    }

    while(1){
        if ((ret = bn_init()) < 0) {
            logs << ERR << "bn_init() error : " << getErrorStr(-ret) << "(#" << -ret << ")\n";
            //exitAI(EXIT_FAILURE);
        }
        else
            break;
    }


    if(roleSetup(true, simu_primary) < 0)
        exitAI(EXIT_FAILURE);

#ifdef SYNC_PROP_IA
    if((ret = bn_intp_sync(role_get_addr(ROLE_PRIM_PROPULSION), 50)) < 0){
        logs << ERR << "FAILED SYNC: " << getErrorStr(-ret) << "(#" << -ret << ")\n";
        exitAI(EXIT_FAILURE);
    }
#endif
#ifdef SYNC_TURRET
    sMsg queryMsg;
    queryMsg.header.destAddr = ADDRI_MAIN_TURRET;
    queryMsg.header.type = E_SYNC_QUERY;
    // beacons
    queryMsg.payload.syncQuery.cfgs[0].type = SYNCTYPE_BEACONS;
    // main AI
    queryMsg.payload.syncQuery.cfgs[1].type = SYNCTYPE_ROLE;
    queryMsg.payload.syncQuery.cfgs[1].role = ROLE_PRIM_AI;
    // prop
    queryMsg.payload.syncQuery.cfgs[2].type = SYNCTYPE_ROLE;
    queryMsg.payload.syncQuery.cfgs[2].role = ROLE_PRIM_PROPULSION;
    // arduino IO
    queryMsg.payload.syncQuery.cfgs[3].type = SYNCTYPE_ADDRESS;
    queryMsg.payload.syncQuery.cfgs[3].addr = ADDRI_MAIN_IO;
    // set sizes
    queryMsg.payload.syncQuery.nb = 4;
    queryMsg.header.size = queryMsg.payload.syncQuery.nb * sizeof(queryMsg.payload.syncQuery.cfgs[0]);
    while (bn_sendAck(&queryMsg)<0);
#endif


    sendPing();

    Env2015::setup();

    setupRobots(simu_primary, holo_primary, hmi_simu_primary, color_primary, eAIState);

    ret = 1;

    logs << INFO << "Initialization is finished";

#ifdef CTRLC_MENU
    signal(SIGINT, intHandler);
    printf("listening, CTRL+C  for menu\n");

    while (!quit){
        int nbTraces,f; //for traceroute display
#else

    // loop
    while (1){
#endif
        usleep(500);

        // check if receiving new messages
        checkInbox(verbose);

        // calls loop functions
        Env2015::loop();
        if (!loopRobots())
            break;

        // calls maintenance function
        statuses.maintenace();
        path.maintenace();
        net.maintenace();

        // menu
#ifdef CTRLC_MENU
        if (menu) {
            quitMenu=0;
            while (!quitMenu) {
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

                switch (cmd) {
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
                        if(ret < 0) {
                            printf("bn_send(E_ROLE_SETUP) error #%i\n", -ret);
                        }
                    }
                    break;
                    case 's' :  //sends debug address to distant node
                    do {
                        printf("enter destination address\n");
                        ret = scanf("%hx",&destAd);
                        if (ret != 1) {
                            printf("error getting destination address\n");
                        }
                    }while(ret != 1);
                    if ( (ret=bn_debugSendAddr(destAd)) > 0) {
                        printf("signalling send\n");
                        quitMenu=1;
                    }
                    else {
                        printf("error while sending : %d\n", ret);

                    }
                    break;
                    case 'p' :
                    do {
                        printf("enter destination address\n");
                        ret = scanf("%hx",&destAd);
                        if (ret != 1) {
                            printf("error getting destination address\n");
                        }
                    }while(ret != 1);
                    printf("ping %hx : %d ms\n",destAd,bn_ping(destAd));
                    break;
                    case 't' :
                    {
                        sTraceInfo *trInfo=NULL;
                        int depth;
                        do {
                            printf("enter destination address\n");
                            ret = scanf("%hx",&destAd);
                            if (ret != 1) {
                                printf("error getting destination address\n");
                            }
                        }while(ret != 1);
                        do {
                            printf("enter depth\n");
                            ret = scanf("%i",&depth);
                            if (ret != 1 || depth <= 0) {
                                printf("error getting depth\n");
                            }
                        }while(ret != 1 || depth <= 0);
                        trInfo = (sTraceInfo *)malloc(depth * sizeof(sTraceInfo));
                        nbTraces=bn_traceroute(destAd,trInfo,depth,1000);
                        for (f=0;f<nbTraces;f++) {
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

    logs << INFO << "bye bye ... \n";

    return EXIT_SUCCESS;
}
