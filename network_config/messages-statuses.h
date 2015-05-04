/*
 * messages-entities.h
 *
 *  Created on: 6 oct. 2014
 *      Author: ludo6431
 */

#ifndef LIB_NETWORK_CONFIG_MESSAGES_STATUSES_H_
#define LIB_NETWORK_CONFIG_MESSAGES_STATUSES_H_

#include <stdint.h>
#include "messages-elements.h"
#include "messages-position.h"

typedef struct __attribute__((packed)){
    uint32_t date;      // synchronized date (µs)
    eElement id :8;
    union{
        // generic way to access position (if present in type)
        struct{
            s2DPosAtt pos;
            s2DPAUncert pos_u;
        };

        // in case of pos.id == ELT_PRIMARY
        struct{
            s2DPosAtt pos;
            s2DPAUncert pos_u;

            s2DSpeed spd;

            uint16_t tid; // trajectory identifier
            uint8_t sid; // step identifier
            uint8_t ssid; // sub-step identifier (0:line, 1:circle)
        } prop_status;

        // in case of pos.id == ELT_ADV_*
        struct{
            s2DPosAtt pos;
            s2DPAUncert pos_u;
        } adv_status;

        // in case of pos.id == ELT_FIRE
        struct{
            s2DPosAtt pos;
            s2DPAUncert pos_u;

            enum{
                FIRE_HORIZ_YELLOW,
                FIRE_HORIZ_RED,
                FIRE_VERTICAL,
                FIRE_OBLIQUE,
                FIRE_VERTICAL_TORCH
            } state :8;
        } fire_status;

        // in case of pos.id == ELT_ZONE
        struct{
            s2DPosAtt pos;
            s2DPAUncert pos_u;
            uint8_t nbpt;
            uint8_t nbfire;
            uint8_t nbtorch;
        } zone_status;
    };
} sGenericStatus;

typedef struct __attribute__((packed)){
    uint8_t nb_obs;
    float r_robot;
    float x_min, x_max, y_min, y_max;
} sObsConfig;

#define MAX_NB_OBSS_PER_MSG (6)
typedef struct __attribute__((packed)){
    uint16_t nb_obs; // must be <= 6 (2 + 8*6 = 50)
    struct __attribute__((packed)){ // 8bytes
        int16_t x; // (LSB=0.1mm)
        int16_t y; // (LSB=0.1mm)
        int16_t r; // (LSB=0.1mm)

        uint8_t moved :4;
        uint8_t active :4;
        uint8_t id; // index in the tab of obstacles
    } obs[];
} sObss;

typedef enum{
    SYNCTYPE_BEACONS,
    SYNCTYPE_ADDRESS,
    SYNCTYPE_ROLE
}eSyncType;

typedef struct __attribute__((packed)){
    uint8_t nb; // must be <= 18
    struct __attribute__((packed)){ // 3bytes
        eSyncType type :8;
        union{
            bn_Address addr;
            uint8_t role;
        };
    } cfgs[];
} sSyncQuery;

typedef enum{
    SYNCSTATUS_KO,
    SYNCSTATUS_ROLE_KO,
    SYNCSTATUS_PING_KO,
    SYNCSTATUS_SYNC_KO,
    SYNCSTATUS_OK,
}eSyncStatus;

typedef struct __attribute__((packed)){
    uint8_t nb; // must be <= 18
    struct __attribute__((packed)){ // 3bytes
        eSyncType type :8;
        union{
            bn_Address addr;
            uint8_t role;
        };
        eSyncStatus status;
    } cfgs[];
} sSyncResponse;

#endif /* LIB_NETWORK_CONFIG_MESSAGES_STATUSES_H_ */
