/*
 * messages.c
 *
 *  Created on: 15 mai 2013
 *      Author: quentin
 */

#include <stdlib.h>

#include "messages.h"

const char *eType2str(E_TYPE elem){
    switch (elem){
    case E_DEBUG :                  return "DEBUG";
    case E_ROLE_SETUP :             return "ROLE_SETUP";
    case E_DATA :                   return "DATA";
    case E_ACK_RESPONSE :           return "ACK_RESPONSE";
    case E_PING :                   return "PING";
    case E_TRACEROUTE_REQUEST :     return "TRACEROUTE_REQUEST";
    case E_TRACEROUTE_RESPONSE :    return "TRACEROUTE_RESPONSE";

    /************************ user types start ************************/
    case E_SYNC_DATA :              return "SYNC_DATA";
    case E_PERIOD :                 return "PERIOD";
    case E_MEASURE :                return "MEASURE";
    case E_TRAJ :                   return "TRAJ";
    case E_ASSERV_STATS :           return "ASSERV_STATS";
    case E_GOAL :                   return "GOAL";
    case E_OBS_CFG :                return "OBS_CFG";
    case E_OBSS :                   return "OBSS" ;
    case E_POS_QUERY :              return "POS_QUERY" ;
    case E_SERVOS :                 return "SERVOS" ;
    case E_IHM_STATUS :             return "IHM_STATUS" ;
    case E_GENERIC_POS_STATUS :     return "GENERIC_POS_STATUS" ;
    case E_POS_STATS :              return "POS_STATS" ;
    case E_TRAJ_ORIENT_EL :         return "TRAJ_ORIENT_EL" ;
    case E_SYNC_QUERY :             return "SYNC_QUERY" ;
    case E_SYNC_RESPONSE :          return "SYNC_RESPONSE" ;
    case E_DO_ABSPOS :              return "DO_ABSPOS" ;
    case E_DONE_ABSPOS :            return "DONE_ABSPOS" ;
    case E_PROP_STOP :              return "PROP_STOP" ;
    case E_TRAJ_POS_SPD_EL :        return "TRAJ_POS_SPD_EL" ;
    /************************ user types stop ************************/

    default :
        return "not an E_TYPE element";
    }

    return NULL;
}
