/*
 * network_cfg.c
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
 * MYADDRD : address of the udp interface of the node
 */

// addresses check
#if MYADDRX && ((MYADDRX & SUBNET_MASK) != SUBNETX)
#error "MYADDRX not on xBee subnet"
#endif
#if MYADDRI && ((MYADDRI & SUBNET_MASK) != SUBNETI)
#error "MYADDRI not on I²C subnet"
#endif
#if MYADDRU && ((MYADDRU & SUBNET_MASK) != SUBNETU1) && ((MYADDRU & SUBNET_MASK) != SUBNETU2)
#error "MYADDRU not on UART subnet"
#endif
#if MYADDRD && ((MYADDRD & SUBNET_MASK) != SUBNETD1) && ((MYADDRD & SUBNET_MASK) != SUBNETD2)
#error "MYADDRD not on UDP subnet"
#endif

#if MYADDRI & 1
#error "I²C address is odd, do you know what you do?"
#endif

sRTableEntry rTable[]={
#if (MYADDRX == ADDRX_MAIN_TURRET || MYADDRI == ADDRI_MAIN_TURRET)
// project "balise_principal"
    {SUBNETU1, {IF_I2C,  ADDRI_MAIN_IO}},
    {SUBNETU2, {IF_I2C,  ADDRI_MAIN_PROP}},
    {SUBNETD1, {IF_I2C,  ADDRI_MAIN_IO}},
    {SUBNETD2, {IF_I2C,  ADDRI_MAIN_PROP}},
    {0x42&(~SUBNET_MASK),{IF_DROP,0}}
#elif (MYADDRI == ADDRI_MAIN_IO || MYADDRU == ADDRU1_MAIN_IO)
// project "arduino_io"
    {SUBNETX,  {IF_I2C,  ADDRI_MAIN_TURRET}},
    {SUBNETU2, {IF_I2C,  ADDRI_MAIN_PROP}},
    {SUBNETD1, {IF_UART, ADDRU1_DBGBRIDGE}},
    {SUBNETD2, {IF_I2C,  ADDRI_MAIN_PROP}},
    {0x42&(~SUBNET_MASK),{IF_DROP,0}}
#elif (MYADDRI == ADDRI_MAIN_PROP || MYADDRU == ADDRU2_MAIN_PROP)
// project "main_prop_axle/lpc"
    {SUBNETX,  {IF_I2C,  ADDRI_MAIN_TURRET}},
    {SUBNETU1, {IF_I2C,  ADDRI_MAIN_IO}},
    {SUBNETD1, {IF_I2C,  ADDRI_MAIN_IO}},
    {SUBNETD2, {IF_UART, ADDRU2_MAIN_AI}},
    {0x42&(~SUBNET_MASK),{IF_DROP,0}}
#elif (MYADDRX == ADDRX_MOBILE_1 || MYADDRX == ADDRX_MOBILE_2 || MYADDRX == ADDRX_DEBUG || MYADDRX == ADDRX_FIX)
// project "balise_mobile" OR debug node
    {SUBNETI,  {IF_XBEE, ADDRX_MAIN_TURRET}},
    {SUBNETU1, {IF_XBEE, ADDRX_MAIN_TURRET}},
    {SUBNETU2, {IF_XBEE, ADDRX_MAIN_TURRET}},
    {SUBNETD1, {IF_XBEE, ADDRX_MAIN_TURRET}},
    {SUBNETD2, {IF_XBEE, ADDRX_MAIN_TURRET}},
    {0x42&(~SUBNET_MASK),{IF_DROP,0}}
#elif (MYADDRD == ADDRD1_DEBUG1 || MYADDRD == ADDRD1_DEBUG2  || MYADDRD == ADDRD1_DEBUG3 || MYADDRD == ADDRD1_MAIN_PROP_SIMU || MYADDRD == ADDRD1_MAIN_AI_SIMU || MYADDRD == ADDRD1_MONITORING)
// project "bn_debug_console" OR project "main_prop_axle/linux" OR project "main_ai" OR project "monitoring_hmi/*"
    {SUBNETX,  {IF_UDP,  ADDRD1_DBGBRIDGE}},
    {SUBNETI,  {IF_UDP,  ADDRD1_DBGBRIDGE}},
    {SUBNETU1, {IF_UDP,  ADDRD1_DBGBRIDGE}},
    {SUBNETU2, {IF_UDP,  ADDRD1_DBGBRIDGE}},
    {SUBNETD2, {IF_UDP,  ADDRD1_DBGBRIDGE}},
    {0x42&(~SUBNET_MASK),{IF_DROP,0}}
#elif (MYADDRU == ADDRU2_MAIN_AI || MYADDRD == ADDRD2_MAIN_AI)
// project "main_ai"
    {SUBNETX,  {IF_UART, ADDRU2_MAIN_PROP}},
    {SUBNETI,  {IF_UART, ADDRU2_MAIN_PROP}},
    {SUBNETU1, {IF_UART, ADDRU2_MAIN_PROP}},
    {SUBNETD1, {IF_UART, ADDRU2_MAIN_PROP}},
    {0x42&(~SUBNET_MASK),{IF_DROP,0}}
#elif (MYADDRU == ADDRU1_DBGBRIDGE || MYADDRD == ADDRD1_DBGBRIDGE)
// project "debug_bridge"
    {SUBNETX,  {IF_UART, ADDRU1_MAIN_IO}},
    {SUBNETI,  {IF_UART, ADDRU1_MAIN_IO}},
    {SUBNETU2, {IF_UART, ADDRU1_MAIN_IO}},
    {SUBNETD2, {IF_UART, ADDRU1_MAIN_IO}},
    {0x42&(~SUBNET_MASK),{IF_DROP,0}}
#else
    {0x42&(~SUBNET_MASK),{IF_DROP,0}}
#warning "The static routing table is empty, you won't be able to route messages!"
#endif
};
