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

#if !defined(MYROLE) || (MYROLE!=0 && MYROLE!=ROLE_DEBUG && MYROLE!=ROLE_PRIM_MONITORING && MYROLE!=ROLE_PRIM_AI && MYROLE!=ROLE_PRIM_PROPULSION && MYROLE!=ROLE_SEC_MONITORING && MYROLE!=ROLE_SEC_AI && MYROLE!=ROLE_SEC_PROPULSION)
#error "MYROLE must be defined and correct in node_cfg.h"
#endif

bn_Address role_addresses[] = {
#if MYROLE != ROLE_DEBUG
        ADDR_DEBUG_DFLT,
#endif
#if MYROLE
#   if MYROLE != ROLE_PRIM_MONITORING
        ADDR_MONITORING_DFLT,
#   endif
#   if MYROLE != ROLE_PRIM_AI
        ADDR_AI_DFLT,
#   endif
#   if MYROLE != ROLE_PRIM_PROPULSION
        ADDR_PROP_DFLT,
#   endif
#endif
};
#define NB_ROLE_ADDRESSES (sizeof(role_addresses)/sizeof(*role_addresses))

sRoleActions role_actions[] = {
        // ROLEMSG_DEBUG messages
        {
                .sendTo={
                    .first = ADDR_DEBUG_DFLT?ROLE_DEBUG:0,
                    .second = 0,
                },
                .relayTo={
                    .n1 = 0,
                    .n2 = 0
                }
        },

#if MYROLE
        // ROLEMSG_PRIM_TRAJ messages
#   if MYROLE == ROLE_PRIM_AI
        {
                .sendTo={
                    .first = ROLE_PRIM_PROPULSION,
                    .second = ROLE_PRIM_MONITORING,
                },
                .relayTo={
                    .n1 = 0,
                    .n2 = 0
                }
        },
#   else
        {
                .sendTo={
                    .first = 0,
                    .second = 0,
                },
                .relayTo={
                    .n1 = 0,
                    .n2 = 0
                }
        },
#   endif

        // ROLEMSG_PRIM_POS messages
#   if MYROLE == ROLE_PRIM_AI
        {
                .sendTo={
                    .first = ROLE_PRIM_PROPULSION,
                    .second = ROLE_PRIM_MONITORING,
                },
                .relayTo={
                    .n1 = ROLE_PRIM_PROPULSION,
                    .n2 = ROLE_PRIM_MONITORING
                }
        },
#   else
        {
                .sendTo={
                    .first = ROLE_PRIM_AI,
                    .second = 0,
                },
                .relayTo={
                    .n1 = 0,
                    .n2 = 0
                }
        },
#   endif
#endif
};
#define NB_ROLE_ACTIONS (sizeof(role_actions)/sizeof(*role_actions))

void role_setup(sMsg *msg){
    int i;
    const sRoleSetupPayload *rs = &msg->payload.roleSetup;

    if(msg->header.type != E_ROLE_SETUP){
        return;
    }

    for(i = 0; i < MIN(18, rs->nb_steps); i++){
        typeof(rs->steps[0]) *s = &rs->steps[i];
        switch(s->step_type){
        case UPDATE_ADDRESS:
            role_set_addr(s->role, s->address);
            break;
        case UPDATE_ACTIONS:
            if(s->type >= 0 && s->type < NB_ROLE_ACTIONS){
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

    if(role > MYROLE){
        role--;
    }

    if(role >= NB_ROLE_ADDRESSES){
        return -1;
    }

    role_addresses[role] = address;

    return 0;
}

// role_get_role(role_get_addr(r)) must be equal to r

bn_Address role_get_addr(uint8_t role){
    if(role == MYROLE){
        return MYADDR;
    }

    if(role > MYROLE){
        role--;
    }

    if(role >= NB_ROLE_ADDRESSES){
        return 0;
    }

    return role_addresses[role];
}

uint8_t role_get_role(bn_Address address){ // TODO update
    switch(address){
    case ADDRD1_DEBUG1:
    case ADDRD1_DEBUG2:
    case ADDRD1_DEBUG3:
    case ADDRX_DEBUG:
        return ROLE_DEBUG;
    case ADDRD1_MONITORING:
        return ROLE_PRIM_MONITORING;
    case ADDRD1_MAIN_AI_SIMU:
    case ADDRD2_MAIN_AI:
    case ADDRU2_MAIN_AI:
        return ROLE_PRIM_AI;
    case ADDRD1_MAIN_PROP_SIMU:
    case ADDRI1_MAIN_PROP:
    case ADDRU2_MAIN_PROP:
        return ROLE_PRIM_PROPULSION;
    default:
        return 0;
    }
}

int role_get_msgclass(E_TYPE msgType, uint8_t destRole, eRoleMsgClass* c){
    switch(msgType){
    default:
        return -1;
    case E_DEBUG:
        switch(destRole){
        case ROLE_DEBUG:
            *c = ROLEMSG_DEBUG;
            break;
        default:
            return -1;
        }
        break;
    case E_TRAJ:
    case E_TRAJ_ORIENT_EL:
        switch(destRole){
        case ROLE_PRIM_MONITORING:
        case ROLE_PRIM_AI:
        case ROLE_PRIM_PROPULSION:
            *c = ROLEMSG_PRIM_TRAJ;
            break;
        case ROLE_SEC_MONITORING:
        case ROLE_SEC_AI:
        case ROLE_SEC_PROPULSION:
            *c = ROLEMSG_SEC_TRAJ;
            break;
        default:
            return -1;
        }
        break;
    case E_POS:
        switch(destRole){
        case ROLE_PRIM_MONITORING:
        case ROLE_PRIM_AI:
        case ROLE_PRIM_PROPULSION:
            *c = ROLEMSG_PRIM_POS;
            break;
        case ROLE_SEC_MONITORING:
        case ROLE_SEC_AI:
        case ROLE_SEC_PROPULSION:
            *c = ROLEMSG_SEC_POS;
            break;
        default:
            return -1;
        }
        break;
    }

    return 0;
}

const char *role_string(uint8_t role){
    switch(role){
    case ROLE_DEBUG:
        return "debug";
    case ROLE_PRIM_MONITORING:
        return "primary monitoring";
    case ROLE_PRIM_AI:
        return "primary ai";
    case ROLE_PRIM_PROPULSION:
        return "primary propulsion";
    case ROLE_SEC_MONITORING:
        return "secondary monitoring";
    case ROLE_SEC_AI:
        return "secondary ai";
    case ROLE_SEC_PROPULSION:
        return "secondary propulsion";
    default:
        return "unknown";
    }
}

int role_send(sMsg *msg, eRoleMsgClass mc){
    int ret = 0;

    if(mc >= NB_ROLE_ACTIONS){
        return -1;
    }
    sRoleActions* act = &role_actions[mc];

    if(act->sendTo.first){
        msg->header.destAddr = role_get_addr(act->sendTo.first);
        if((ret = bn_send(msg)) < 0){
            return ret;
        }
    }
    if(act->sendTo.second){
        msg->header.destAddr = role_get_addr(act->sendTo.second);
        if((ret = bn_send(msg)) < 0){
            return ret;
        }
    }

    return 0;
}

int role_sendAck(sMsg *msg, eRoleMsgClass mc){
    int ret = 0;

    if(mc >= NB_ROLE_ACTIONS){
        return -1;
    }
    sRoleActions* act = &role_actions[mc];

    if(act->sendTo.first){
        msg->header.destAddr = role_get_addr(act->sendTo.first);
        if((ret = bn_sendAck(msg)) < 0){
            return ret;
        }
    }
    if(act->sendTo.second){
        msg->header.destAddr = role_get_addr(act->sendTo.second);
        if((ret = bn_sendAck(msg)) < 0){
            return ret;
        }
    }

    return 0;
}

int role_sendRetry(sMsg *msg, eRoleMsgClass mc, int retries){
    int ret = 0;

    if(mc >= NB_ROLE_ACTIONS){
        return -1;
    }
    sRoleActions* act = &role_actions[mc];

    if(act->sendTo.first){
        msg->header.destAddr = role_get_addr(act->sendTo.first);
        if((ret = bn_sendRetry(msg, retries)) < 0){
            return ret;
        }
    }
    if(act->sendTo.second){
        msg->header.destAddr = role_get_addr(act->sendTo.second);
        if((ret = bn_sendRetry(msg, retries)) < 0){
            return ret;
        }
    }

    return 0;
}

#if MYROLE

int role_relay(sMsg *msg){
    bn_Address dest_addr_save = msg->header.destAddr;
    uint8_t src_role = role_get_role(msg->header.srcAddr);
    uint8_t dest_role = role_get_role(msg->header.destAddr);
    int ret = 0;

    if(!src_role || !dest_role){
        return 0;
    }

    eRoleMsgClass mc;
    if((ret = role_get_msgclass(msg->header.type, dest_role, &mc)) < 0){
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
    }
    if(act->relayTo.n2 == src_role){
        msg->header.destAddr = role_get_addr(act->relayTo.n1);
        if((ret = bn_send(msg)) < 0){
            return ret;
        }
    }

    msg->header.destAddr = dest_addr_save;

    return 0;
}

#endif
