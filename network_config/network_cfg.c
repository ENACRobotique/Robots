/*
 * netrwork_cfg.c
 *
 *  Created on: 28 mai 2013
 *      Author: quentin
 */

#include "network_cfg.h"
#include "node_cfg.h"

/*Remark : requires a "node.cfg.h" in the project, containing :
 * MYADDRX : adress of the xbee interface of the node
 * MYADDRI : address of the i2c interface of the node
 * MYADDRU : address of the uart interface of the node
 * MYADDRD : address of the UDP interface of the node
 */

#if (MYADDRX == ADDRX_MAIN || MYADDRI == ADDRI_MAIN_TURRET)
sRTableEntry rTable[]={
    {0x42&(~SUBNET_MASK),{IF_DROP,0}}
};
#elif (MYADDRI == ADDRI_MAIN_IO)
sRTableEntry rTable[]={
    {SUBNETX, {IF_I2C, ADDRI_MAIN_TURRET}},
    {0x42&(~SUBNET_MASK),{IF_DROP,0}}
};
#elif (MYADDRI == ADDRI_MAIN_PROP)
sRTableEntry rTable[]={
    {SUBNETX, {IF_I2C, ADDRI_MAIN_TURRET}},
    {SUBNETU_1, {IF_I2C, ADDRI_BRIDGE}},
    {0x42&(~SUBNET_MASK),{IF_DROP,0}}
};
#elif (MYADDRX == ADDRX_MOBILE_1)
sRTableEntry rTable[]={
    {0x42&(~SUBNET_MASK),{IF_DROP,0}}
};
#elif  (MYADDRX == ADDRX_MOBILE_2)
sRTableEntry rTable[]={
    {0x42&(~SUBNET_MASK),{IF_DROP,0}}
};
#elif (MYADDRX == ADDRX_REMOTE_IA)
sRTableEntry rTable[]={
    {SUBNETI_MAIN, {IF_XBEE, ADDRX_MAIN}},
    {0x42&(~SUBNET_MASK),{IF_DROP,0}}
};
#elif (MYADDRX == ADDRX_DEBUG)
sRTableEntry rTable[]={
    {SUBNETI_MAIN, {IF_XBEE, ADDRX_MAIN}},
    {0x42&(~SUBNET_MASK),{IF_DROP,0}}
};
#elif (MYADDRX == 0 && MYADDRI == ADDRI_MAIN_TURRET) //for tests purposes only
sRTableEntry rTable[]={
    {0x42&(~SUBNET_MASK),{IF_DROP,0}}
};
#elif (MYADDRD == ADDRD_DEBUG || MYADDRD == ADDRD_MAIN_PROP_SIMU)
sRTableEntry rTable[]={
    {SUBNETI_MAIN, {IF_UDP, ADDRD_DBGBRIDGE}},
    {SUBNETX, {IF_UDP, ADDRD_DBGBRIDGE}},
    {0x42&(~SUBNET_MASK),{IF_DROP,0}}
};
#elif (MYADDRD == ADDRD_DBGBRIDGE)
sRTableEntry rTable[]={
    {SUBNETI_MAIN, {IF_XBEE, ADDRX_MAIN}},
    {0x42&(~SUBNET_MASK),{IF_DROP,0}}
};
#endif
