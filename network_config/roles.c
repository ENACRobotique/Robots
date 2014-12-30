/*
 * roles.c
 *
 *  Created on: 5 mars 2014
 *      Author: ludo6431
 */

#include <string.h>
//#include <stdio.h> // printf()

#include "../static/communication/botNet/shared/botNet_core.h"
#include "network_cfg.h"

#include "roles.h"

#if !defined(MYROLE) || (MYROLE!=0 && MYROLE!=ROLE_MONITORING && MYROLE!=ROLE_AI && MYROLE!=ROLE_PROPULSION && MYROLE!=ROLE_DEBUG)
#error "MYROLE must be defined and correct in node_cfg.h"
#endif

#if MYROLE != ROLE_MONITORING
bn_Address addr_role_mon = ADDR_MONITORING_DFLT;
#endif
#if MYROLE != ROLE_AI
bn_Address addr_role_ai = ADDR_AI_DFLT;
#endif
#if MYROLE != ROLE_PROPULSION
bn_Address addr_role_prop = ADDR_PROP_DFLT;
#endif
#if MYROLE != ROLE_DEBUG
bn_Address addr_role_dbg = ADDR_DEBUG_DFLT;
#endif

sRoleActions role_actions[3] = {
#if MYROLE == ROLE_AI
        { // E_TRAJ messages
                .sendTo.first = ROLE_PROPULSION,
                .sendTo.second = ROLE_MONITORING,
                .relayTo.n1 = 0,
                .relayTo.n2 = 0
        },
#else
        { // E_TRAJ messages
                .sendTo.first = 0,
                .sendTo.second = 0,
                .relayTo.n1 = 0,
                .relayTo.n2 = 0
        },
#endif

#if MYROLE == ROLE_AI
        { // E_POS messages
                .sendTo.first = ROLE_PROPULSION,
                .sendTo.second = ROLE_MONITORING,
                .relayTo.n1 = ROLE_PROPULSION,
                .relayTo.n2 = ROLE_MONITORING
        },
#else
        { // E_POS messages
                .sendTo.first = ROLE_AI,
                .sendTo.second = 0,
                .relayTo.n1 = 0,
                .relayTo.n2 = 0
        },
#endif

        { // E_DEBUG messages
                .sendTo.first = ADDR_DEBUG_DFLT?ROLE_DEBUG:0,
                .sendTo.second = 0,
                .relayTo.n1 = 0,
                .relayTo.n2 = 0
        },
};
#define ACT_MSG_TRAJ (&role_actions[0])
#define ACT_MSG_POS  (&role_actions[1])
#define ACT_MSG_DBG  (&role_actions[2])

void role_setup(sMsg *msg){
    int i;
    const sRoleSetupPayload *rs = &msg->payload.roleSetup;

    if(msg->header.type != E_ROLE_SETUP){
        return;
    }

    for(i = 0; i < MIN(13, rs->nb_steps); i++){
        typeof(rs->steps[0]) *s = &rs->steps[i];
        switch(s->step){
        case UPDATE_ADDRESS:
            role_set_addr(s->role, s->address);
//            printf("auto update address %hhu %hx\n", s->role, s->address);
            break;
        case UPDATE_ACTIONS:
//            printf("auto update actions %s (=%hhu)\n", eType2str(s->type), s->type);
            switch(s->type){
            case E_TRAJ:
                memcpy((void*)ACT_MSG_TRAJ, (void*)&s->actions, sizeof(*ACT_MSG_TRAJ));
                break;
            case E_POS:
                memcpy((void*)ACT_MSG_POS, (void*)&s->actions, sizeof(*ACT_MSG_POS));
                break;
            case E_DEBUG:
                memcpy((void*)ACT_MSG_DBG, (void*)&s->actions, sizeof(*ACT_MSG_DBG));
                break;
            default:
                break;
            }
            break;
        default:
            break;
        }
    }
}

int role_set_addr(uint8_t role, bn_Address address){
    switch(role){
    case ROLE_MONITORING:
#if MYROLE != ROLE_MONITORING
        addr_role_mon = address;
        break;
#else
        return -1;
#endif
    case ROLE_AI:
#if MYROLE != ROLE_AI
        addr_role_ai = address;
        break;
#else
        return -1;
#endif
    case ROLE_PROPULSION:
#if MYROLE != ROLE_PROPULSION
        addr_role_prop = address;
        break;
#else
        return -1;
#endif
    case ROLE_DEBUG:
#if MYROLE != ROLE_DEBUG
        addr_role_dbg = address;
        break;
#else
        return -1;
#endif
    default:
        break;
    }
    return 0;
}

bn_Address role_get_addr(uint8_t role){
    switch(role){
    // monitoring
    case ROLE_MONITORING:
#if MYROLE != ROLE_MONITORING
        return addr_role_mon;
#else
        return MYADDR;
#endif
    // ai
    case ROLE_AI:
#if MYROLE != ROLE_AI
        return addr_role_ai;
#else
        return MYADDR;
#endif
    // propulsion
    case ROLE_PROPULSION:
#if MYROLE != ROLE_PROPULSION
        return addr_role_prop;
#else
        return MYADDR;
#endif
    // debug
    case ROLE_DEBUG:
#if MYROLE != ROLE_DEBUG
        return addr_role_dbg;
#else
        return MYADDR;
#endif
    default:
        return 0;
    }
}

uint8_t role_get_role(bn_Address address){
    switch(address){
    // monitoring
    case ADDRD1_MONITORING:
        return ROLE_MONITORING;
    // ai
    case ADDRD1_MAIN_AI_SIMU:
    case ADDRD2_MAIN_AI:
    case ADDRU2_MAIN_AI:
        return ROLE_AI;
    // propulsion
    case ADDRD1_MAIN_PROP_SIMU:
    case ADDRI1_MAIN_PROP:
    case ADDRU2_MAIN_PROP:
        return ROLE_PROPULSION;
    // debug
    case ADDRD1_DEBUG1:
    case ADDRD1_DEBUG2:
    case ADDRD1_DEBUG3:
    case ADDRX_DEBUG:
        return ROLE_DEBUG;
    default:
        return 0;
    }
}

const char *role_string(uint8_t role){
    switch(role){
    case ROLE_MONITORING:
        return "monitoring";
    case ROLE_AI:
        return "ai";
    case ROLE_PROPULSION:
        return "propulsion";
    case ROLE_DEBUG:
        return "debug";
    default:
        return "unknown";
    }
}

int role_send(sMsg *msg){
    int ret = 0;

#define SEND_BLOCK(act) \
    do{                                                                     \
        if((act)->sendTo.first){                                            \
            msg->header.destAddr = role_get_addr((act)->sendTo.first);      \
            ret = bn_send(msg);                                             \
            if(ret < 0)                                                     \
                break;                                                      \
        }                                                                   \
        if((act)->sendTo.second){                                           \
            msg->header.destAddr = role_get_addr((act)->sendTo.second);     \
            ret = bn_send(msg);                                             \
            if(ret < 0)                                                     \
                break;                                                      \
        }                                                                   \
    }while(0)

    switch(msg->header.type){
    case E_TRAJ:
        SEND_BLOCK(ACT_MSG_TRAJ);
        break;
    case E_POS:
        SEND_BLOCK(ACT_MSG_POS);
        break;
    case E_DEBUG:
        SEND_BLOCK(ACT_MSG_DBG);
        break;
    default:
        break;
    }

#undef SEND_BLOCK

    return ret;
}

int role_sendAck(sMsg *msg){
    int ret = 0;

#define SEND_BLOCK(act) \
    do{                                                                     \
        if((act)->sendTo.first){                                            \
            msg->header.destAddr = role_get_addr((act)->sendTo.first);      \
            ret = bn_sendAck(msg);                                          \
            if(ret < 0)                                                     \
                break;                                                      \
        }                                                                   \
        if((act)->sendTo.second){                                           \
            msg->header.destAddr = role_get_addr((act)->sendTo.second);     \
            ret = bn_sendAck(msg);                                          \
            if(ret < 0)                                                     \
                break;                                                      \
        }                                                                   \
    }while(0)

    switch(msg->header.type){
    case E_TRAJ:
        SEND_BLOCK(ACT_MSG_TRAJ);
        break;
    case E_POS:
        SEND_BLOCK(ACT_MSG_POS);
        break;
    case E_DEBUG:
        SEND_BLOCK(ACT_MSG_DBG);
        break;
    default:
        break;
    }

#undef SEND_BLOCK

    return ret;
}

int role_sendRetry(sMsg *msg, int retries){
    int ret = 0;

#define SEND_BLOCK(act) \
    do{                                                                     \
        if((act)->sendTo.first){                                            \
            msg->header.destAddr = role_get_addr((act)->sendTo.first);      \
            ret = bn_sendRetry(msg, retries);                                          \
            if(ret < 0)                                                     \
                break;                                                      \
        }                                                                   \
        if((act)->sendTo.second){                                           \
            msg->header.destAddr = role_get_addr((act)->sendTo.second);     \
            ret = bn_sendRetry(msg, retries);                                          \
            if(ret < 0)                                                     \
                break;                                                      \
        }                                                                   \
    }while(0)

    switch(msg->header.type){
    case E_TRAJ:
        SEND_BLOCK(ACT_MSG_TRAJ);
        break;
    case E_POS:
        SEND_BLOCK(ACT_MSG_POS);
        break;
    case E_DEBUG:
        SEND_BLOCK(ACT_MSG_DBG);
        break;
    default:
        break;
    }

#undef SEND_BLOCK

    return ret;
}

#if MYROLE

int role_relay(sMsg *msg){
    bn_Address dest_save = msg->header.destAddr;
    uint8_t msg_src_role = role_get_role(msg->header.srcAddr);
    int ret = 0;

    if(!msg_src_role || !role_get_role(msg->header.destAddr)){
        return 0;
    }

#define RELAY_BLOCK(act) \
    do{                                                                     \
        if((act)->relayTo.n1 == msg_src_role){                              \
            msg->header.destAddr = role_get_addr((act)->relayTo.n2);        \
            ret = bn_send(msg);                                             \
            if(ret < 0)                                                     \
                break;                                                      \
        }                                                                   \
        if((act)->relayTo.n2 == msg_src_role){                              \
            msg->header.destAddr = role_get_addr((act)->relayTo.n1);        \
            ret = bn_send(msg);                                             \
            if(ret < 0)                                                     \
                break;                                                      \
        }                                                                   \
    }while(0)

    switch(msg->header.type){
    case E_TRAJ:
        RELAY_BLOCK(ACT_MSG_TRAJ);
        break;
    case E_POS:
        RELAY_BLOCK(ACT_MSG_POS);
        break;
    case E_DEBUG:
        RELAY_BLOCK(ACT_MSG_DBG);
        break;
    default:
        break;
    }
    msg->header.destAddr = dest_save;

#undef RELAY_BLOCK

    return ret;
}

#endif
