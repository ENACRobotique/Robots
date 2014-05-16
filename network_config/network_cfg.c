/*
 * netrwork_cfg.c
 *
 *  Created on: 28 mai 2013
 *      Author: quentin
 */

#include "network_cfg.h"
#include "node_cfg.h"

/*Remark : requires a "node.cfg.h" in the project, containing :
 * MYADDRX : address of the xbee interface of the node
 * MYADDRI : address of the i2c interface of the node
 * MYADDRU : address of the uart interface of the node
 * MYADDRD : address of the UDP interface of the node
 */

// addresses check
#if MYADDRX && ((MYADDRX & SUBNET_MASK) != SUBNETX)
#error "MYADDRX not on xBee subnet"
#endif
#if MYADDRI && ((MYADDRI & SUBNET_MASK) != SUBNETI_MAIN)
#error "MYADDRI not on I²C subnet"
#endif
#if MYADDRU && ((MYADDRU & SUBNET_MASK) != SUBNETU_DEBUG)
#error "MYADDRU not on UART subnet"
#endif
#if MYADDRD && ((MYADDRD & SUBNET_MASK) != SUBNETD_DEBUG)
#error "MYADDRD not on UDP subnet"
#endif

#if MYADDRI & 1
#error "I²C address is odd, do you know what you do?"
#endif

#if (MYADDRX == ADDRX_MAIN_TURRET || MYADDRI == ADDRI_MAIN_TURRET)
sRTableEntry rTable[]={
    {SUBNETD_DEBUG, {IF_I2C, ADDRI_MAIN_IO}},
    {SUBNETU_DEBUG, {IF_I2C, ADDRI_MAIN_IO}},
    {0x42&(~SUBNET_MASK),{IF_DROP,0}}
};
#elif (MYADDRI == ADDRI_MAIN_IO || MYADDRU == ADDRU_MAIN_IO)
sRTableEntry rTable[]={
    {SUBNETX, {IF_I2C, ADDRI_MAIN_TURRET}},
    {SUBNETD_DEBUG, {IF_UART, ADDRU_DBGBRIDGE}},
    {0x42&(~SUBNET_MASK),{IF_DROP,0}}
};
#elif (MYADDRI == ADDRI_MAIN_PROP)
sRTableEntry rTable[]={
    {SUBNETX, {IF_I2C, ADDRI_MAIN_TURRET}},
    {SUBNETD_DEBUG, {IF_I2C, ADDRI_MAIN_IO}},
    {SUBNETU_DEBUG, {IF_I2C, ADDRI_MAIN_IO}},
    {0x42&(~SUBNET_MASK),{IF_DROP,0}}
};
#elif (MYADDRX == ADDRX_MOBILE_1 || MYADDRX == ADDRX_MOBILE_2 || MYADDRX == ADDRX_FIX)
sRTableEntry rTable[]={
    {SUBNETD_DEBUG, {IF_XBEE, ADDRX_MAIN_TURRET}},
    {SUBNETI_MAIN,  {IF_XBEE, ADDRX_MAIN_TURRET}},
    {SUBNETU_DEBUG, {IF_XBEE, ADDRX_MAIN_TURRET}},
    {0x42&(~SUBNET_MASK),{IF_DROP,0}}
};
#elif (MYADDRX == ADDRX_REMOTE_IA)
sRTableEntry rTable[]={
    {SUBNETI_MAIN, {IF_XBEE, ADDRX_MAIN_TURRET}},
    {0x42&(~SUBNET_MASK),{IF_DROP,0}}
};
#elif (MYADDRX == ADDRX_DEBUG)
sRTableEntry rTable[]={
    {SUBNETI_MAIN, {IF_XBEE, ADDRX_MAIN_TURRET}},
    {0x42&(~SUBNET_MASK),{IF_DROP,0}}
};
#elif (MYADDRX == 0 && MYADDRI == ADDRI_MAIN_TURRET) //for tests purposes only
sRTableEntry rTable[]={
    {0x42&(~SUBNET_MASK),{IF_DROP,0}}
};
#elif (MYADDRD == ADDRD_DEBUG2 || MYADDRD == ADDRD_DEBUG1 || MYADDRD == ADDRD_MAIN_PROP_SIMU || MYADDRD == ADDRD_MAIN_IA_SIMU || MYADDRD == ADDRD_MONITORING)
sRTableEntry rTable[]={
    {SUBNETI_MAIN, {IF_UDP, ADDRD_DBGBRIDGE}},
    {SUBNETU_DEBUG, {IF_UDP, ADDRD_DBGBRIDGE}},
    {SUBNETX, {IF_UDP, ADDRD_DBGBRIDGE}},
    {0x42&(~SUBNET_MASK),{IF_DROP,0}}
};
#elif (MYADDRD == ADDRD_DBGBRIDGE || MYADDRU == ADDRU_DBGBRIDGE)
sRTableEntry rTable[]={
    {SUBNETX, {IF_UART, ADDRU_MAIN_IO}},
    {SUBNETI_MAIN, {IF_UART, ADDRU_MAIN_IO}},
//    {SUBNETI_MAIN, {IF_XBEE, ADDRU_MAIN_IO}},
    {0x42&(~SUBNET_MASK),{IF_DROP,0}}
};
#endif
