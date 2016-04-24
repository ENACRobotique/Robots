/*
 * messages-arm.h
 *
 *  Created on: 24 april. 2016
 *      Author: Yoann Solana
 */

#ifndef LIB_NETWORK_CONFIG_MESSAGES_ARM_H_
#define LIB_NETWORK_CONFIG_MESSAGES_ARM_H_

#include <stdint.h>
#include "messages-objects.h"


typedef enum eTypeArmMsgs{
    arm_goalTarget,
    eTypeArmMsgs_max
}eTypeArmMsgs;

typedef enum eTypeConstruction{ // WARNING: the order must be same that in file ObjsConstruct.h in arm_hrrr project.
    construct_tower11,
    construct_tower21,
    construct_wall222,
    construct_wall322,
    eTypeConstructionMax
}eTypeConstruction;

typedef struct __attribute__((packed)){
// position in reference frame (specified in field "frame")
    uint8_t idArm;      // the arm
    float x;            // (cm)
    float y;            // (cm)
    float z;            // (cm)
    float beta;         // (rad)
} sGoalTarget; // 3D position & end effector angle

typedef struct __attribute__((packed)){
// position in reference frame (specified in field "frame")
    uint8_t idArm;      // the arm
    uint8_t conf;            // id of a static configuration
} sStaticConf; // Put the arm in a specific static configuration

typedef struct __attribute__((packed)){
// position in reference frame (specified in field "frame")
    uint8_t idArm;      // the arm
    eTypeObj objType;   // Type of the object
    uint8_t idObj;      // Id of the object
    float x;            // center of the object(cm)
    float y;            // center of the object(cm)
    float z;            // center of the object(cm)
    float beta;         // (rad)
} sPickPutObject; // to grasp or place an object

typedef struct __attribute__((packed)){
// position in reference frame (specified in field "frame")
    uint8_t idConstruct;      // the construction
    eTypeConstruction type;   // type of the construction
    float x;                  // center of the construction(cm)
    float y;                  // center of the construction(cm)
    float z;                  // center of the construction(cm)
    float beta;               // (rad)
} sMakeConstruct; // to grasp or place an object

#endif /* LIB_NETWORK_CONFIG_MESSAGES_ARM_H_ */
