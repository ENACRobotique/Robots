/*
 * roles.c
 *
 *  Created on: 5 mars 2014
 *      Author: ludo6431
 */

#include <string.h>
#ifdef ARCH_X86_LINUX
#   include <assert.h>
#endif
//#include <stdio.h> // printf()

#include "../static/communication/botNet/shared/botNet_core.h"
#include "network_cfg.h"

#include "roles.h"

#if !defined(MYROLE) || MYROLE < 0 || MYROLE > LAST_ROLE
#   error "MYROLE must be defined and correct in node_cfg.h"
#endif

bn_Address role_addresses[] = { // using   ROLE_*-1   as index in this array
    ADDR_DEBUG_DFLT,
    ADDR_MONITORING_DFLT,
    ADDR_PRIM_AI_DFLT,          // prim ai
    ADDR_PRIM_PROP_DFLT,        // prim prop
    ADDR_PRIM_VIDEO_DFLT,       // prim video
                                // sec ai
                                // sec prop
};
#define NB_ROLE_ADDRESSES (sizeof(role_addresses)/sizeof(*role_addresses))

// defines default route and relays for handled message classes
sRoleActions role_actions[] = {
        // ROLEMSG_DEBUG messages
        {
                .sendTo={
                    .first = ROLE_DEBUG,
                    .second = 0,
                },
                .relayTo={
                    .n1 = 0,
                    .n2 = 0
                }
        },

        // ROLEMSG_PRIM_TRAJ messages
        {
#if MYROLE == ROLE_PRIM_AI
                .sendTo={
                    .first = ROLE_PRIM_PROPULSION,
                    .second = ROLE_MONITORING,
                },
                .relayTo={
                    .n1 = 0,
                    .n2 = 0
                }
#else
                .sendTo={
                    .first = 0,
                    .second = 0,
                },
                .relayTo={
                    .n1 = 0,
                    .n2 = 0
                }
#endif
        },

        // ROLEMSG_PRIM_POS messages
        {
#if MYROLE == ROLE_PRIM_AI
                .sendTo={
                    .first = ROLE_PRIM_PROPULSION,
                    .second = ROLE_MONITORING,
                },
                .relayTo={
                    .n1 = ROLE_PRIM_PROPULSION,
                    .n2 = ROLE_MONITORING
                }
#elif MYROLE == ROLE_PRIM_PROPULSION
                .sendTo={
                    .first = ROLE_PRIM_AI,
                    .second = 0,
                },
                .relayTo={
                    .n1 = 0,
                    .n2 = 0
                }
#else
                .sendTo={
                    .first = 0,
                    .second = 0,
                },
                .relayTo={
                    .n1 = 0,
                    .n2 = 0
                }
#endif
        },

        // ROLEMSG_PRIM_ABSPOS messages
        {
#if MYROLE == ROLE_PRIM_VIDEO
                .sendTo={
                    .first = ROLE_PRIM_PROPULSION,
                    .second = ROLE_PRIM_AI,
                },
                .relayTo={
                    .n1 = 0,
                    .n2 = 0
                }
#else
                .sendTo={
                    .first = 0,
                    .second = 0,
                },
                .relayTo={
                    .n1 = 0,
                    .n2 = 0
                }
#endif
        },
};
#define NB_ROLE_ACTIONS (sizeof(role_actions)/sizeof(*role_actions))

void role_setup(sMsg *msg){
    int i;
    const sRoleSetupPayload *rs = &msg->payload.roleSetup;

    if(msg->header.type != E_ROLE_SETUP){
        return;
    }

    for(i = 0; i < MIN(13, rs->nb_steps); i++){
        typeof(rs->steps[0]) *s = &rs->steps[i];
        switch(s->step_type){
        case UPDATE_ADDRESS:
            role_set_addr(s->role, s->address);
            break;
        case UPDATE_ACTIONS:
            if(/*s->type >= 0 &&*/ s->type < NB_ROLE_ACTIONS){
                memcpy((void*)&role_actions[s->type], (void*)&s->actions, sizeof(*role_actions));
            }
            break;
        default:
            break;
        }
    }
}

int role_set_addr(uint8_t role, bn_Address address){
    if(role == MYROLE){
        return -1;
    }

#ifdef ARCH_X86_LINUX
    assert(role_get_role(address) == role);
#endif

    if(!role || role > NB_ROLE_ADDRESSES){
        return -1;
    }

    role_addresses[role - 1] = address;

    return 0;
}

bn_Address role_get_addr(uint8_t role){
    if(role == MYROLE){
        return MYADDR;
    }

    if(role > NB_ROLE_ADDRESSES){
        return 0;
    }

    bn_Address address = role_addresses[role - 1];

#ifdef ARCH_X86_LINUX
    assert(role_get_role(address) == role);
#endif

    return address;
}

uint8_t role_get_role(bn_Address address){
    switch(address){
    case ADDRD1_DEBUG1:
    case ADDRD1_DEBUG2:
    case ADDRD1_DEBUG3:
    case ADDRX_DEBUG:
    case ADDRI_DEBUG:
        return ROLE_DEBUG;

    case ADDRD1_MONITORING:
        return ROLE_MONITORING;

    case ADDRD1_MAIN_AI_SIMU:
    case ADDRD2_MAIN_AI:
        return ROLE_PRIM_AI;

    case ADDRD1_MAIN_PROP_SIMU:
    case ADDRI_MAIN_PROP:
    case ADDRU2_MAIN_PROP:
        return ROLE_PRIM_PROPULSION;

    case ADDRD1_MAIN_VIDEO_SIMU:
    case ADDRD2_MAIN_VIDEO:
        return ROLE_PRIM_VIDEO;

    default:
        return 0;
    }
}

const char *role_string(uint8_t role){
    switch(role){
    case ROLE_DEBUG:
        return "debug";
    case ROLE_MONITORING:
        return "monitoring";
    case ROLE_PRIM_AI:
        return "primary ai";
    case ROLE_PRIM_PROPULSION:
        return "primary propulsion";
    case ROLE_PRIM_VIDEO:
        return "primary video";
    case ROLE_SEC_AI:
        return "secondary ai";
    case ROLE_SEC_PROPULSION:
        return "secondary propulsion";
    default:
        return "unknown";
    }
}

int role_send(sMsg *msg, eRoleMsgClass mc){
    int ret = 0, ret2 = 0;

    if(mc >= NB_ROLE_ACTIONS){
        return -1;
    }
    sRoleActions* act = &role_actions[mc];

    if(act->sendTo.first){
        msg->header.destAddr = role_get_addr(act->sendTo.first);
        if((ret = bn_send(msg)) < 0){
            return ret;
        }
        ret2 += ret > 0;
    }
    if(act->sendTo.second){
        msg->header.destAddr = role_get_addr(act->sendTo.second);
        if((ret = bn_send(msg)) < 0){
            return ret;
        }
        ret2 += ret > 0;
    }

    return ret2;
}

int role_sendAck(sMsg *msg, eRoleMsgClass mc){
    int ret = 0, ret2 = 0;

    if(mc >= NB_ROLE_ACTIONS){
        return -1;
    }
    sRoleActions* act = &role_actions[mc];

    if(act->sendTo.first){
        msg->header.destAddr = role_get_addr(act->sendTo.first);
        if((ret = bn_sendAck(msg)) < 0){
            return ret;
        }
        ret2 += ret > 0;
    }
    if(act->sendTo.second){
        msg->header.destAddr = role_get_addr(act->sendTo.second);
        if((ret = bn_sendAck(msg)) < 0){
            return ret;
        }
        ret2 += ret > 0;
    }

    return ret2;
}

int role_sendRetry(sMsg *msg, eRoleMsgClass mc, int retries){
    int ret = 0, ret2 = 0;

    if(mc >= NB_ROLE_ACTIONS){
        return -1;
    }
    sRoleActions* act = &role_actions[mc];

    if(act->sendTo.first){
        msg->header.destAddr = role_get_addr(act->sendTo.first);
        if((ret = bn_sendRetry(msg, retries)) < 0){
            return ret;
        }
        ret2 += ret > 0;
    }
    if(act->sendTo.second){
        msg->header.destAddr = role_get_addr(act->sendTo.second);
        if((ret = bn_sendRetry(msg, retries)) < 0){
            return ret;
        }
        ret2 += ret > 0;
    }

    return ret2;
}

#if MYROLE

int role_get_msgclass(E_TYPE msgType, uint8_t __attribute__((__unused__)) srcRole, eRoleMsgClass* c){
    int ret = 1;

    switch(msgType){
    case E_DEBUG:
        *c = ROLEMSG_DEBUG;
        ret = 0;
        break;
    case E_TRAJ:
    case E_TRAJ_ORIENT_EL:
#if MYROLE == ROLE_PRIM_AI || MYROLE == ROLE_PRIM_PROPULSION
        *c = ROLEMSG_PRIM_TRAJ;
        ret = 0;
#elif MYROLE == ROLE_SEC_AI || MYROLE == ROLE_SEC_PROPULSION
        *c = ROLEMSG_SEC_TRAJ;
        ret = 0;
#elif MYROLE == ROLE_MONITORING
        switch(srcRole){
        case ROLE_PRIM_AI:
        case ROLE_PRIM_PROPULSION:
            *c = ROLEMSG_PRIM_TRAJ;
            ret = 0;
            break;
        case ROLE_SEC_AI:
        case ROLE_SEC_PROPULSION:
            *c = ROLEMSG_SEC_TRAJ;
            ret = 0;
            break;
        default:
            break;
        }
#endif
        break;
    case E_GENERIC_POS_STATUS:
#if MYROLE == ROLE_PRIM_AI || MYROLE == ROLE_PRIM_PROPULSION
        *c = ROLEMSG_PRIM_POS;
        ret = 0;
#elif MYROLE == ROLE_SEC_AI || MYROLE == ROLE_SEC_PROPULSION
        *c = ROLEMSG_SEC_POS;
        ret = 0;
#elif MYROLE == ROLE_MONITORING
        switch(srcRole){
        case ROLE_PRIM_AI:
        case ROLE_PRIM_PROPULSION:
            *c = ROLEMSG_PRIM_POS;
            ret = 0;
            break;
        case ROLE_SEC_AI:
        case ROLE_SEC_PROPULSION:
            *c = ROLEMSG_SEC_POS;
            ret = 0;
            break;
        default:
            break;
        }
#endif
        break;
    case E_DO_ABSPOS:
    case E_DONE_ABSPOS:
        *c = ROLEMSG_PRIM_ABSPOS;
        ret = 0;
        break;
    default:
        break;
    }

    return ret;
}

int role_relay(sMsg *msg){
    bn_Address dest_addr_save = msg->header.destAddr;
    uint8_t src_role = role_get_role(msg->header.srcAddr);
    int ret = 0, ret2 = 0;

    if(!src_role){
        return 0;
    }

    eRoleMsgClass mc;
    if((ret = role_get_msgclass(msg->header.type, src_role, &mc))){
        return ret;
    }
    if(mc >= NB_ROLE_ACTIONS){
        return -1;
    }
    sRoleActions* act = &role_actions[mc];

    if(act->relayTo.n1 == src_role){
        msg->header.destAddr = role_get_addr(act->relayTo.n2);
        if((ret = bn_send(msg)) < 0){
            return ret;
        }
        ret2 += ret > 0;
    }
    if(act->relayTo.n2 == src_role){
        msg->header.destAddr = role_get_addr(act->relayTo.n1);
        if((ret = bn_send(msg)) < 0){
            return ret;
        }
        ret2 += ret > 0;
    }

    msg->header.destAddr = dest_addr_save;

    return ret2;
}

#endif

