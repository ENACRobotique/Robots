/*
 * messages.c
 *
 *  Created on: 15 mai 2013
 *      Author: quentin
 */

#include "messages.h"



#ifdef __cplusplus
extern "C" {
#endif


char *eType2str(E_TYPE elem){

    switch (elem){

    case E_DATA : return "DATA";
    case E_DEBUG : return "DEBUG";
    case E_DEBUG_ADDR : return "DEBUG_ADDR";
    case E_MEASURE : return "MEASURE";
    case E_PERIOD : return "PERIOD";
    case E_SWITCH_CHANNEL : return "SWITCH_CHANNEL";
    case E_SYNC_EXPECTED_TIME : return "SYNC_EXPECTED";
    case E_SYNC_OK : return "SYNC_OK";
    case E_RAW : return "RAW";

    default :
        return "not an E_TYPE element";
    }

    return 0;
}


#ifdef __cplusplus
}
#endif
