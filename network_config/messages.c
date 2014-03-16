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
    case E_SWITCH_CHANNEL :         return "SWITCH_CHANNEL";
    case E_SYNC_EXPECTED_TIME :     return "SYNC_EXPECTED";
    case E_SYNC_OK :                return "SYNC_OK";
    case E_PERIOD :                 return "PERIOD";
    case E_MEASURE :                return "MEASURE";
    case E_TRAJ :                   return "TRAJ";
    case E_POS :                    return "POS";
    case E_SERIAL_DUMP :            return "SERIAL_DUMP";
    case E_ASSERV_STATS :           return "ASSERV_STATS";
    case E_GOAL :                   return "GOAL";
    case E_OBS_CFG :                return "OBS_CFG";
    case E_OBSS :                   return "OBSS" ;
    /************************ user types stop ************************/

    default :
        return "not an E_TYPE element";
    }

    return NULL;
}
