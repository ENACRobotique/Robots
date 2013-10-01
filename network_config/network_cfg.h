/*
 * network_cfg.h
 *
 *  Created on: 28 mai 2013
 *      Author: quentin
 */

#ifndef NETWORK_CFG_H_
#define NETWORK_CFG_H_


#include <stdint.h>
#include "message_header.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef BIT
#define BIT(a) (1<<(a))
#endif



#define DEVICE_ADDR_SIZE 8      //in bits, on a 16 bits adress. Must equal the larger size of the adresse in the different subnetworks

//masks
    #define SUBNET_MASK (0xff<<DEVICE_ADDR_SIZE)  //on a 16-bits adress
	#define DEVICEX_MASK ( BIT(DEVICE_ADDR_SIZE)-1 )
	#define DEVICEI_MASK ( BIT(DEVICE_ADDR_SIZE)-1 )
    #define ADDRI_MASK  (0xff)      //on a 16-bits adress, xbee devices
    #define ADDRX_MASK  (0xff)      //on a 16-bits adress, i2c devices

//subnet adresses
    #define SUBNETX         (1<<DEVICE_ADDR_SIZE)
    #define SUBNETI_MAIN    (2<<DEVICE_ADDR_SIZE)


//xbee adresses
    #define ADDRX_MAIN      ( BIT(0) | SUBNETX )
    #define ADDRX_FIX       ( BIT(1) | SUBNETX )
    #define ADDRX_MOBILE_1  ( BIT(2) | SUBNETX )
    #define ADDRX_MOBILE_2  ( BIT(3) | SUBNETX )
    #define ADDRX_SECOND    ( BIT(4) | SUBNETX )
    #define ADDRX_DEBUG     ( BIT(5) | SUBNETX )
    #define ADDRX_REMOTE_IA ( BIT(6) | SUBNETX )
    #define ADDRX_BROADCAST ( 0xff   | SUBNETX )

//I2C adresses
    //subnet MAIN
    #define ADDRI_MAIN_TURRET   (2 | SUBNETI_MAIN)
    #define ADDRI_MAIN_PROP     (4 | SUBNETI_MAIN)
    #define ADDRI_MAIN_GLASS    (6 | SUBNETI_MAIN)
    #define ADDRI_MAIN_CANDLE   (8 | SUBNETI_MAIN)



/* Interface enum
 *
 */
typedef enum{
	IF_XBEE,
	IF_I2C,
	IF_UART,
	IF_LOCAL, 	//virtal interface, describing local node. A message send to "self" should be popped out and "given" to the node trought the sb_receive() api
	IF_DROP,	//virtual interface, equivalent to /dev/null in linux


	IF_COUNT
}E_IFACE;

/* routing table entry
 * the routing of a message "msg", with destination address "destAddr", coming from the interface "interface" is the following :
 * if (the destination is on one of our subnetworks){
 * 		if (the message has not already been red on the latter subnet){
 * 			send message on the appropriate subnet;
 * 			return;
 * 			}
 * 		}
 *
 * while(we are not at the end of the routing table){
 *      if (destination's subnet==routing table entry subnet){
 *      	send to ifTo;
 *      	return
 *      }
 *      go to next routing table entry
 * }
 * perform default action (send to default)
 *
 * /!\ the last entry MUST be {0x42&(~SUBNET_MASK),default interface,default address}
 */

typedef struct {
    E_IFACE ifTo;
    sb_Address nextHop;
}sRouteInfo;

typedef struct{
    sb_Address destSubnet;
    sRouteInfo nextHop;
}sRTableEntry;




extern sRTableEntry rTable[];

#ifdef __cplusplus
}
#endif


#endif /* NETWORK_CFG_H_ */
