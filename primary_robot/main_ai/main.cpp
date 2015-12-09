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
#include "MainAITypes.h"
#include "Nodes.h"

#define OPT_PRIMARY_MAIN_AI         1000
#define OPT_PRIMARY_ARDUINO_IO      1001
#define OPT_PRIMARY_TURRET          1002
#define OPT_PRIMARY_HMI             1003
#define OPT_PRIMARY_PROP            1004
#define OPT_PRIMARY_DEBUB_BRIDGE    1005


void exitAI(int status){
    cout << "\033[0m";
    exit(status);
}

void usage(char *cl) {
    printf("main ia\n");
    printf("Usage:\n\t%s [options]\n", cl);
    printf("Options:\n");
    printf("\t--mode,     -m        AI mode {slave | auto | prog}\n");
    printf("\t--log-file, -f        output log file of received messages (overwritten)\n");
    printf("\t--verbose,  -v        increases verbosity\n");
    printf("\t--quiet,    -q        not verbose\n");
    printf("\t--info,     -i        print the current configuration\n");
    printf("\t--help,     -h, -?    prints this help\n");
    printf("\t--primary-mainAI      {real | simu | none}\n");
    printf("\t--primary-arduinoIO   {real | simu | none}\n");
    printf("\t--primary-beacon      {real | simu | none}\n");
    printf("\t--primary-hmi         {real | simu | none}\n");
    printf("\t--primary-prop        {real | simu | none}\n");
    printf("\t--primary-debugBridge {real | simu | none}\n");

}

ostream& operator<<(ostream& os, eSyncType st){
    switch(st){
        case SYNCTYPE_ADDRESS:
            os << "ADDRESS";
            break;
        case SYNCTYPE_BEACONS:
            os << "BEACONS";
            break;
        case SYNCTYPE_ROLE:
            os << "ROLE";
            break;
        default:
            os << "UNKNOWN";
            break;
    }
    return os;
}

envNode getEnvNode(char *optarg){
    if (!strcasecmp(optarg, "real")) {
        return envNode::REAL;
    }
    else if (!strcasecmp(optarg, "simu")) {
        return envNode::SIMU;
    }
    else if (!strcasecmp(optarg, "none")) {
        return envNode::NONE;
    }

    logs << ERR << "Invalid argument";
    return envNode::NONE;
}

int main(int argc, char **argv) {
    int ret;
    eAIState_t eAIState = E_AI_SLAVE;
    bool simu_primary = true;
    bool holo_primary = true;
    bool hmi_simu_primary = true;
    eColor_t color_primary = eColor_t::GREEN;
    Nodes nodes;

    //default configuration
    nodesNet.addNodes(nameNode::MAIN_AI    , nodes.getCfg(nameNode::MAIN_AI   , envNode::SIMU));
    nodesNet.addNodes(nameNode::HMI        , {envNode::SIMU, 0, ROLE_UNDEFINED});
    nodesNet.addNodes(nameNode::PROP       , nodes.getCfg(nameNode::PROP      , envNode::SIMU));
    nodesNet.addNodes(nameNode::MONITORING , nodes.getCfg(nameNode::MONITORING, envNode::SIMU));

    // arguments parsing
    while (1) {
        static struct option long_options[] = {
                { "mode"                , required_argument, NULL, 'm'                      },
                { "log-file"            , required_argument, NULL, 'f'                      },
                { "quiet"               , no_argument      , NULL, 'q'                      },
                { "help"                , no_argument      , NULL, 'h'                      },
                { "verbose"             , no_argument      , NULL, 'v'                      },
                { "info"                , no_argument      , NULL, 'i'                      },
                { "primary-mainAI"      , required_argument, NULL, OPT_PRIMARY_MAIN_AI      },
                { "primary-arduinoIO"   , required_argument, NULL, OPT_PRIMARY_ARDUINO_IO   },
                { "primary-beacon"      , required_argument, NULL, OPT_PRIMARY_TURRET       },
                { "primary-hmi"         , required_argument, NULL, OPT_PRIMARY_HMI          },
                { "primary-prop"        , required_argument, NULL, OPT_PRIMARY_PROP         },
                { "primary-debugBridge" , required_argument, NULL, OPT_PRIMARY_DEBUB_BRIDGE },
                { "color"               , required_argument, NULL, 'c'                      },
                { "secondary"           , required_argument, NULL, 's'                      },
                { "adv-primary"         , required_argument, NULL, 'a'                      },
                { "adv-secondary"       , required_argument, NULL, 'b'                      },
                { NULL, 0, NULL, 0 }
        };


        int c = getopt_long(argc, argv, "m:f:p:s:a:b:ivqh?", long_options, NULL);
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
            case 'c':
                if (!strcasecmp(optarg, "green")) {
                    color_primary = eColor_t::GREEN;
                }
                else if (!strcasecmp(optarg, "yellow")) {
                    color_primary = eColor_t::PURPLE;
                }
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
            case 'i':
                nodesNet.print();
                logs << "\nPress a key to continue...";
                getchar();
                break;
            case OPT_PRIMARY_MAIN_AI:
                nodesNet.addNodes(nameNode::MAIN_AI, nodes.getCfg(nameNode::MAIN_AI, getEnvNode(optarg)));
                break;
            case OPT_PRIMARY_ARDUINO_IO:
                nodesNet.addNodes(nameNode::ARDUINO_IO, nodes.getCfg(nameNode::ARDUINO_IO, getEnvNode(optarg)));
                break;
            case OPT_PRIMARY_TURRET:
                nodesNet.addNodes(nameNode::TURRET, nodes.getCfg(nameNode::TURRET, getEnvNode(optarg)));
                break;
            case OPT_PRIMARY_HMI:
                if(getEnvNode(optarg) == envNode::SIMU)
                    nodesNet.addNodes(nameNode::HMI, {envNode::SIMU, 0, ROLE_UNDEFINED});
                else if(getEnvNode(optarg) == envNode::REAL)
                    nodesNet.addNodes(nameNode::HMI, {envNode::REAL, 0, ROLE_UNDEFINED});
                break;
            case OPT_PRIMARY_PROP:
                nodesNet.addNodes(nameNode::PROP, nodes.getCfg(nameNode::PROP, getEnvNode(optarg)));
                simu_primary=false;
                hmi_simu_primary=false;
                //role_set_addr(ROLE_PRIM_PROPULSION, ADDRU2_MAIN_PROP);
                break;
            case OPT_PRIMARY_DEBUB_BRIDGE:
                nodesNet.addNodes(nameNode::DEBUG_BRIDGE, nodes.getCfg(nameNode::DEBUG_BRIDGE, getEnvNode(optarg)));
                break;
            default:
                logs << ERR << "?? getopt returned character code 0" << c << "??";
                /* no break */
            case 'h':
            case '?':
                usage(argv[0]);
                exitAI(EXIT_FAILURE);
                break;
        }
    }

    // network initialization
    while ((ret = bn_attach(E_ROLE_SETUP, role_setup)) < 0)
        logs << ERR << "bn_attach() error : " << getErrorStr(-ret) << "(#" << -ret << ")\n";
    /*
    if(/nodes._nodes[nameNode::TURRET].env==envNode::REAL/ 1){
       while((ret = bn_intp_install()) < 0)
            logs << ERR << "bn_intp_install() error : " << getErrorStr(-ret) << "(#" << -ret << ")\n";
    }
*/
    while((ret = bn_init()) < 0)
        logs << ERR << "bn_init() error : " << getErrorStr(-ret) << "(#" << -ret << ")\n";


    //sendPing(nodes);

    if(roleSetup(true, simu_primary, true) < 0)
        exitAI(EXIT_FAILURE);

   // if(syncSetup(nodes.nodes[nameNode::TURRET].env==envNode::REAL?true:false) < 0)
    if(syncSetup(false) < 0)
        exitAI(EXIT_FAILURE);

    logs << INFO << "Fin synchro";
    Env2016::setup();

    setupRobots(simu_primary, holo_primary, hmi_simu_primary, color_primary, eAIState);

    ret = 1;

    logs << INFO << "Initialization is finished";


        ihm.sendIhm(IHM_LED, LED_GREEN);

    while(1){
        if(servo.initServo() == 1)
            break;

        sleep(1);
        }
    // loop
    while (1){

        usleep(500);

        // check if receiving new messages
        inbox.checkInbox();

        // calls loop functions
        Env2016::loop();
        if (!loopRobots())
            break;

        // calls maintenance function
        statuses.maintenace();
        path.maintenace();
        net.maintenace();

        // menu

    }
//TODO Stop robot
    logs << INFO << "bye bye ... \n";

    return EXIT_SUCCESS;
}
