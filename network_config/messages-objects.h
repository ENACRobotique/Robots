/*
 * messages-position.h
 *
 *  Created on: 6 oct. 2014
 *      Author: ludo6431
 */

#ifndef LIB_NETWORK_CONFIG_MESSAGES_OBJECTS_H_
#define LIB_NETWORK_CONFIG_MESSAGES_OBJECTS_H_

#include <stdint.h>
#include "messages-position.h"

typedef enum eObjectType{
    SAND_CUB,
    SAND_CYL,
    SAND_CON,
    SHELL_GREEN,
    SHELL_PURPLE,
    SHELL_WHITE
} eObjectType;

typedef struct sObject{
    eObjectType type;
    s3DPos pose;
    s3DPos pose_u;
} sObject; // does not excess 56 bytes


#endif /* LIB_NETWORK_CONFIG_MESSAGES_OBJECTS_H_ */
