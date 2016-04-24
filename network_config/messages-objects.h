/*
 * messages-arm.h
 *
 *  Created on: 24 april. 2016
 *      Author: Yoann Solana
 */

#ifndef LIB_NETWORK_CONFIG_MESSAGES_OBJECTS_H_
#define LIB_NETWORK_CONFIG_MESSAGES_OBJECTS_H_

#include <stdint.h>


typedef enum eTypeObj{
    sandCube,
    sandCyl,
    sandCone,
    eTypeObjMax
}eTypeObj;

typedef enum eStateObj{
    addObj,
    removeObj,
    eStateObjMax
}eStateObj;

typedef struct __attribute__((packed)){
// position in reference frame (specified in field "frame")
    eTypeObj type;      // type of the object
    uint8_t idObj;         // id of the object
    float x;            // (cm)
    float y;            // (cm)
    float z;            // (cm)
    float beta;         // (rad)
    eStateObj state;    // State of the object
}sObject; // Describe an object and its configuration


#endif /* LIB_NETWORK_CONFIG_MESSAGES_OBJECTS_H_ */
