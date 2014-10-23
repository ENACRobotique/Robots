/*
 * messages-elements.c
 *
 *  Created on: 6 oct. 2014
 *      Author: ludo6431
 */


#include "messages-elements.h"

int elementHasPosition(eElement id){
    switch(id){
    case ELT_PRIMARY:
    case ELT_SECONDARY:
    case ELT_ADV_PRIMARY:
    case ELT_ADV_SECONDARY:
    case ELT_FIRE:
    case ELT_ZONE:
        return 1;
    default:
        return 0;
    }
}
