/*
 * messages.c
 *
 *  Created on: 15 mai 2013
 *      Author: quentin
 */

#include "messages.h"



char *eType2str(E_TYPE elem){

    switch (elem){

    case E_DATA : return "DATA";
    case E_DEBUG : return "DEBUG";
    case E_MEASURE : return "MEASURE";
    case E_PERIOD : return "PERIOD";
    case E_SWITCH_CHANNEL : return "SWITCH_CHANNEL";
    case E_SYNC_EXPECTED_TIME : return "SYNC_EXPECTED";
    case E_SYNC_OK : return "SYNC_OK";

    default :
        return "not an E_TYPE element";
    }

    return 0;
}

