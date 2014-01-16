/*
 * messages.c
 *
 *  Created on: 15 mai 2013
 *      Author: quentin
 */

#include "messages.h"



char *eType2str(E_TYPE elem){

    switch (elem){

    case E_DEBUG : return "DEBUG";
    case E_DATA : return "DATA";


    default :
        return "not an E_TYPE element";
    }

    return 0;
}

