/*
 * network_cfg.h
 *
 *  Created on: 28 mai 2013
 *      Author: quentin
 */

#ifndef NETWORK_CFG_H_
#define NETWORK_CFG_H_


#include <stdint.h>
#include "../static/communication/botNet/shared/message_header.h"


#ifdef __cplusplus
extern "C" {
#endif

#include "roles.h"

#define DEVICE_ADDR_SIZE 8      //in bits, on a 16 bits address. Must equal the larger size of the address in the different subnetworks

//masks
    #define SUBNET_MASK (0xff<<DEVICE_ADDR_SIZE)  //on a 16-bits address
    #define DEVICEX_MASK ( BIT(DEVICE_ADDR_SIZE)-1 )
    #define DEVICEI_MASK ( BIT(DEVICE_ADDR_SIZE)-1 )
    #define DEVICEU_MASK ( BIT(DEVICE_ADDR_SIZE)-1 )
    #define DEVICED_MASK ( BIT(DEVICE_ADDR_SIZE)-1 )
    #define ADDRX_MASK  (0xff)      //on a 16-bits address, Xbee devices
    #define ADDRI_MASK  (0xff)      //on a 16-bits address, I²C devices
    #define ADDRU_MASK  (0xff)      //on a 16-bits address, uart devices
    #define ADDRD_MASK  (0xff)      //on a 16-bits address, udp devices

//subnet addresses
    #define SUBNETX         (1<<DEVICE_ADDR_SIZE | BCAST_SUBNET)
    #define SUBNETI_MAIN    (2<<DEVICE_ADDR_SIZE)
    #define SUBNETU1_DEBUG  (3<<DEVICE_ADDR_SIZE)
    #define SUBNETD1_DEBUG  (4<<DEVICE_ADDR_SIZE)
    #define SUBNETU2_MAIN   (5<<DEVICE_ADDR_SIZE)
    #define SUBNETD2_MAIN   (6<<DEVICE_ADDR_SIZE)

//xbee addresses
    #define ADDRX_MAIN_TURRET   ( BIT(0) | SUBNETX )
    #define ADDRX_FIX           ( BIT(1) | SUBNETX )
    #define ADDRX_MOBILE_1      ( BIT(2) | SUBNETX )
    #define ADDRX_MOBILE_2      ( BIT(3) | SUBNETX )
    #define ADDRX_SECOND        ( BIT(4) | SUBNETX )
    #define ADDRX_DEBUG         ( BIT(5) | SUBNETX )
    #define ADDRX_DBGBRIDGE     ( BIT(7) | SUBNETX )
    #define ADDRX_BROADCAST     ( 0xff   | SUBNETX )

//I2C addresses (least significant bit of I²C addresses must be unused)
    //subnet MAIN
    #define ADDRI_MAIN_TURRET   ( (1<<1) | SUBNETI_MAIN )
    #define ADDRI_MAIN_PROP     ( (2<<1) | SUBNETI_MAIN )
    #define ADDRI_MAIN_IO       ( (3<<1) | SUBNETI_MAIN )
    #define ADDRI_DBGBRIDGE     ( (4<<1) | SUBNETI_MAIN )

//UART addresses (SUBNETU1_DEBUG)
    #define ADDRU1_DBGBRIDGE    ( 1 | SUBNETU1_DEBUG )
    #define ADDRU1_MAIN_IO      ( 2 | SUBNETU1_DEBUG )

//UART addresses (SUBNETU2_MAIN)
    #define ADDRU2_MAIN_PROP    ( 1 | SUBNETU2_MAIN )
    #define ADDRU2_MAIN_AI      ( 2 | SUBNETU2_MAIN )

//UDP addresses (SUBNETD1_DEBUG)
    #define ADDRD1_DEBUG1               ( 1 | SUBNETD1_DEBUG )
    #define ADDRD1_DEBUG2               ( 2 | SUBNETD1_DEBUG )
    #define ADDRD1_DEBUG3               ( 3 | SUBNETD1_DEBUG )
    #define ADDRD1_DBGBRIDGE            ( 4 | SUBNETD1_DEBUG )
    #define ADDRD1_MONITORING           ( 5 | SUBNETD1_DEBUG )
    #define ADDRD1_MAIN_PROP_SIMU       ( 6 | SUBNETD1_DEBUG )
    #define ADDRD1_MAIN_IA_SIMU         ( 7 | SUBNETD1_DEBUG )

//UDP addresses (SUBNETD2_MAIN)
    #define ADDRD2_MAIN_AI              ( 1 | SUBNETD2_MAIN )
    #define ADDRD2_MAIN_VIDEO           ( 2 | SUBNETD2_MAIN )

//default debug address :
    #define ADDR_MONITORING_DFLT    (ADDRD1_MONITORING)
    #define ADDR_IA_DFLT            (ADDRD1_MAIN_IA_SIMU)
#if 1
    #define ADDR_PROP_DFLT          (ADDRD1_MAIN_PROP_SIMU)
#else
    #define ADDR_PROP_DFLT          (ADDRI_MAIN_PROP)
#endif
    #define ADDR_DEBUG_DFLT         (ADDRX_DEBUG)

/* Interface enum
 *
 */
typedef enum{
    IF_XBEE,
    IF_I2C,
    IF_UART,
    IF_UDP,
    IF_LOCAL,     //virtual interface, describing local node. A message send to "self" should be popped out and "given" to the node through the sb_receive() api
    IF_DROP,    //virtual interface, equivalent to /dev/null in linux

    IF_COUNT
}E_IFACE;

/* routing table entry
 * the routing of a message "msg", with destination address "destAddr", coming from the interface "interface" is the following :
 * if (the destination is on one of our subnetworks){
 *         if (the message has not already been red on the latter subnet){
 *             send message on the appropriate subnet;
 *             return;
 *             }
 *         }
 *
 * while(we are not at the end of the routing table){
 *      if (destination's subnet==routing table entry subnet){
 *          send to ifTo;
 *          return
 *      }
 *      go to next routing table entry
 * }
 * perform default action (send to default)
 *
 * /!\ the last entry MUST be {0x42&(~SUBNET_MASK),default interface,default address}
 */

typedef struct {
    E_IFACE ifTo;
    bn_Address nextHop;
}sRouteInfo;

typedef struct{
    bn_Address destSubnet;
    sRouteInfo nextHop;
}sRTableEntry;


extern sRTableEntry rTable[];

#ifdef __cplusplus
}
#endif


#endif /* NETWORK_CFG_H_ */
