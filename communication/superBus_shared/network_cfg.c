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

#if ( MYADDRX == ADDRX_MAIN && MYADDRI == ADDRI_MAIN_TURRET)
sRTableEntry rTable[]={
        {SUBNETX,IF_XBEE},
        {0x42&(~SUBNET_MASK),IF_XBEE}
};
#elif ( MYADDRX == ADDRX_MOBILE_1)

#elif  (MYADDRX == ADDRX_MOBILE_2)
sRTableEntry rTable[]={
        {0x42&(~SUBNET_MASK),IF_DROP}
};

#elif (MYADDRX == ADDRX_DEBUG)
sRTableEntry rTable[]={
        {0x42&(~SUBNET_MASK),IF_DROP}
};
#endif
