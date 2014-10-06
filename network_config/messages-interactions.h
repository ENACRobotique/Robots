/*
 * messages-interactions.h
 *
 *  Created on: 6 oct. 2014
 *      Author: ludo6431
 */

#ifndef LIB_NETWORK_CONFIG_MESSAGES_INTERACTIONS_H_
#define LIB_NETWORK_CONFIG_MESSAGES_INTERACTIONS_H_

#include <stdint.h>

typedef enum{
    SERVO_PRIM_DOOR,
    SERVO_PRIM_FIRE1,
    SERVO_PRIM_FIRE2,
    SERVO_PRIM_ARM_LEFT,
    SERVO_PRIM_ARM_RIGHT,

    NUM_E_SERVO
} eServos;

typedef struct __attribute__((packed)){
    uint16_t nb_servos; // must be <=18
    struct __attribute__((packed)){
            eServos id :8; // identifier of the servomotor
            uint16_t us; // servo setpoint in µs
    } servos[];
} sServos;

typedef enum{
    IHM_STARTING_CORD,
    IHM_MODE_SWICTH,
    IHM_LED,
    IHM_LIMIT_SWITCH_RIGHT,
    IHM_LIMIT_SWITCH_LEFT
} eIhmElement;

typedef struct __attribute__((packed)){
    uint16_t nb_states; // must be <=18
    struct __attribute__((packed)){
        eIhmElement id :8; // identifier of the ihm element
        uint16_t state; // status
    } states[];
} sIhmStatus;


#endif /* LIB_NETWORK_CONFIG_MESSAGES_INTERACTIONS_H_ */
