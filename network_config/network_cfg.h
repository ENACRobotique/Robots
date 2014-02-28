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

#ifndef BIT
#define BIT(a) (1<<(a))
#endif


#define DEVICE_ADDR_SIZE 8      //in bits, on a 16 bits address. Must equal the larger size of the address in the different subnetworks

//masks
    #define SUBNET_MASK (0xff<<DEVICE_ADDR_SIZE)  //on a 16-bits address
    #define DEVICEX_MASK ( BIT(DEVICE_ADDR_SIZE)-1 )
    #define DEVICEI_MASK ( BIT(DEVICE_ADDR_SIZE)-1 )
    #define DEVICEU_MASK ( BIT(DEVICE_ADDR_SIZE)-1 )
    #define DEVICED_MASK ( BIT(DEVICE_ADDR_SIZE)-1 )
    #define ADDRX_MASK  (0xff)      //on a 16-bits address, i2c devices
    #define ADDRI_MASK  (0xff)      //on a 16-bits address, xbee devices
    #define ADDRU_MASK  (0xff)
    #define ADDRD_MASK  (0xff)

//subnet addresses
    #define SUBNETX         (1<<DEVICE_ADDR_SIZE)
    #define SUBNETI_MAIN    (2<<DEVICE_ADDR_SIZE)
    #define SUBNETU_DEBUG   (3<<DEVICE_ADDR_SIZE)
    #define SUBNETD_DEBUG   (4<<DEVICE_ADDR_SIZE)

//xbee addresses
    #define ADDRX_MAIN      ( BIT(0) | SUBNETX )
    #define ADDRX_FIX       ( BIT(1) | SUBNETX )
    #define ADDRX_MOBILE_1  ( BIT(2) | SUBNETX )
    #define ADDRX_MOBILE_2  ( BIT(3) | SUBNETX )
    #define ADDRX_SECOND    ( BIT(4) | SUBNETX )
    #define ADDRX_DEBUG     ( BIT(5) | SUBNETX )
    #define ADDRX_REMOTE_IA ( BIT(6) | SUBNETX )
    #define ADDRX_DBGBRIDGE ( BIT(7) | SUBNETX )
    #define ADDRX_BROADCAST ( 0xff   | SUBNETX )

//I2C addresses (least significant bit of I²C addresses must be unused)
    //subnet MAIN
    #define ADDRI_MAIN_TURRET   ( (1<<1) | SUBNETI_MAIN )
    #define ADDRI_MAIN_PROP     ( (2<<1) | SUBNETI_MAIN )
    #define ADDRI_MAIN_IO       ( (3<<1) | SUBNETI_MAIN )
    #define ADDRI_DBGBRIDGE     ( (4<<1) | SUBNETI_MAIN )

//UART addresses
    #define ADDRU_DBGBRIDGE ( 1 | SUBNETU_DEBUG )
    #define ADDRU_DEBUG     ( 2 | SUBNETU_DEBUG )

//UDP addresses
    #define ADDRD_DBGBRIDGE        ( 1 | SUBNETD_DEBUG )
    #define ADDRD_DEBUG            ( 2 | SUBNETD_DEBUG )
    #define ADDRD_MAIN_PROP_SIMU   ( 3 | SUBNETD_DEBUG )
    #define ADDRD_MAIN_IA_SIMU     ( 4 | SUBNETD_DEBUG )

//default debug address :
    #define ADDR_DEBUG_DFLT 0



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
