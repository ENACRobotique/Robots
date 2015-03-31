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

typedef struct __attribute__((packed)){
    uint16_t nb_steps; // must be <=13 to fit in a sMsg payload (2+4*13=54)
    struct{ // 4bytes
        enum{
            UPDATE_ADDRESS,
            UPDATE_ACTIONS
        } step :8;
        union{
            struct __attribute__((packed)){
                uint8_t role;
                bn_Address address;
            };
            struct __attribute__((packed)){
                E_TYPE type :8;
                sRoleActions actions;
            };
        };
    } steps[];
} sRoleSetupPayload;


#endif /* LIB_NETWORK_CONFIG_MESSAGES_NETWORK_H_ */
