/*
 * messages-network.h
 *
 *  Created on: 6 oct. 2014
 *      Author: ludo6431
 */

#ifndef LIB_NETWORK_CONFIG_MESSAGES_NETWORK_H_
#define LIB_NETWORK_CONFIG_MESSAGES_NETWORK_H_

#include <stdint.h>
#include "messages.h"
#include "../static/communication/botNet/shared/message_header.h"

//Specific payloads
typedef struct __attribute__((packed)){
    uint8_t     index;  // index of the current message (n)
    uint32_t    time;  // date of sending of the current message (n) in the master's clock
    uint32_t    prevTime;  // date of sending of the previous message (n-1) in the master's clock
}sINTP;

// a role of 0 is considered unavailable
// can't be an enum because we do things at preprocessor time
#define ROLE_UNDEFINED          (0)
#define ROLE_DEBUG              (1)
// primary roles
#define ROLE_PRIM_MONITORING    (2)
#define ROLE_PRIM_AI            (3)
#define ROLE_PRIM_PROPULSION    (4)
// secondary roles
#define ROLE_SEC_MONITORING     (5)
#define ROLE_SEC_AI             (6)
#define ROLE_SEC_PROPULSION     (7)

typedef struct __attribute__((packed)){ // 2bytes
    struct __attribute__((packed)){
        uint8_t first  :4;
        uint8_t second :4;
    } sendTo;
    struct __attribute__((packed)){
        uint8_t n1 :4;
        uint8_t n2 :4;
    } relayTo;
} sRoleActions;

typedef enum {
    ROLEMSG_DEBUG,
    ROLEMSG_PRIM_TRAJ,
    ROLEMSG_PRIM_POS,
    ROLEMSG_SEC_TRAJ,
    ROLEMSG_SEC_POS,
} eRoleMsgClass;

typedef struct __attribute__((packed)){
    uint16_t nb_steps; // must be <=18 to fit in a sMsg payload (2+3*18=56)
    struct __attribute__((packed)){ // 3bytes
        enum{
            UPDATE_ADDRESS,
            UPDATE_ACTIONS
        } step_type :4;
        union{
            // case update address
            struct __attribute__((packed)){
                uint8_t role :4;
                bn_Address address;
            };
            // case update actions
            struct __attribute__((packed)){
                eRoleMsgClass type :4;
                sRoleActions actions;
            };
        };
    } steps[];
} sRoleSetupPayload;


#endif /* LIB_NETWORK_CONFIG_MESSAGES_NETWORK_H_ */
