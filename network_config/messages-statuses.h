/*
 * messages-entities.h
 *
 *  Created on: 6 oct. 2014
 *      Author: ludo6431
 */

#ifndef LIB_NETWORK_CONFIG_MESSAGES_STATUSES_H_
#define LIB_NETWORK_CONFIG_MESSAGES_STATUSES_H_

#include <messages-elements.h>
#include <messages-position.h>
#include <stdint.h>
#include <sys/types.h>

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

            float speed; // (cm/s)

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

#endif /* LIB_NETWORK_CONFIG_MESSAGES_STATUSES_H_ */
