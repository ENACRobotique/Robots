/*
 * netrwork_cfg.c
 *
 *  Created on: 28 mai 2013
 *      Author: quentin
 */

#include "network_cfg.h"
#include "params.h"

/*Remark : requires a "params.h" in the project, containing :
 * MYADDRX : adress of the xbee interface of the node
 * MYADDRI : address of the i2c interface of ne node
 */

#if (MYADDRX == ADDRX_MAIN || MYADDRI == ADDRI_MAIN_TURRET)
sRTableEntry rTable[]={
        {0x42&(~SUBNET_MASK),{IF_DROP,0}}
};
#elif (MYADDRX == 0 && MYADDRI == ADDRI_MAIN_CANDLE)
sRTableEntry rTable[]={
        {SUBNETX, {IF_I2C, ADDRI_MAIN_TURRET}},
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
#elif (MYADDRX == ADDRX_DEBUG)
sRTableEntry rTable[]={
        {SUBNETI_MAIN, {IF_XBEE, ADDRX_MAIN}},
        {0x42&(~SUBNET_MASK),{IF_DROP,0}}
};
#elif (MYADDRX == 0 && MYADDRI == ADDRI_MAIN_TURRET) //for tests purposes only
sRTableEntry rTable[]={
        {0x42&(~SUBNET_MASK),{IF_DROP,0}}
};
#endif
