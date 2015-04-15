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
    SERVO_PRIM_GLASS1_HOLD,
    SERVO_PRIM_GLASS1_RAISE,
    SERVO_PRIM_GLASS2_HOLD,
    SERVO_PRIM_GLASS2_RAISE,
    SERVO_PRIM_GLASS3_HOLD,
    SERVO_PRIM_GLASS3_RAISE,

    SERVO_PRIM_LIFT1_UP,
    SERVO_PRIM_LIFT1_DOOR,
    SERVO_PRIM_LIFT1_HOLD,
    SERVO_PRIM_LIFT2_UP,
    SERVO_PRIM_LIFT2_DOOR,
    SERVO_PRIM_LIFT2_HOLD,

    SERVO_PRIM_CORN1_RAMP,
    SERVO_PRIM_CORN2_RAMP,
    SERVO_PRIM_CORN_DOOR,

    NUM_E_SERVO
} eServos;

typedef struct __attribute__((packed)){
    uint16_t nb_servos; // must be <=10
    struct __attribute__((packed)){ // 5 bytes
            eServos id :8; // identifier of the servomotor
            float angle; // servo setpoint in degrees (0-*)
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
