/*
 * node_cfg.h
 *
 *  Created on: 1 oct. 2013
 *      Author: quentin
 */

#ifndef NODE_CFG_H_
#define NODE_CFG_H_

#include "network_cfg.h"

// Addresses of the current node (0 means no such interface attached to this device)
// These addresses MUST be first defined in network_cfg.h
#define MYADDRX 0                   // Xbee address . Remember to include the Xbee_API_shared lib and Xbee_API_XXXARCH lib in the build
#define MYADDRI ADDRI_MAIN_PROP   // I2C address. Remember to include the relevant architecture libraries in the build if needed.
#define MYADDRU 0                   // UART address. Remember to include the relevant architecture libraries in the build if needed.
#define MYADDRD 0
#define MYADDR (MYADDRX?:MYADDRI?:MYADDRU?:MYADDRD)

#define MYROLE 0
// MYROLE must be equal to role_get_role(MYADDR)

// Superbus parameters
#define BN_INC_MSG_BUF_SIZE 4           // size of the global incoming buffer (requires RAM space : SB_INC_MSG_BUF_SIZE*SB_MAX_PDU Bytes(cf message.h) )
#define BN_MAX_RETRIES 2                // number of retries at the superBus level (if the level 2 send fail)

// Xbee for superbus parameters
#define BN_WAIT_XBEE_SND_FAIL   25000   //experimentally set, time in µs before we consider a send frame not "statused" if no status frame has been received.
#define BN_ACK_TIMEOUT 1000 // in ms

// Xbee parameters
#define XBEE_WAITFRAME_TIMEOUT 10       // timeout for a waited frame, in µs
#define XBEE_READBYTE_TIMEOUT 100000    // timeout for a expected byte, in µs

// Default debug address override. If 0, no debug message will be send (until a debug signalling message is received and properly handled).
//#undef  ADDR_DEBUG_DFLT
//#define ADDR_DEBUG_DFLT ADDRX_DEBUG

#endif /* NODE_CFG_H_ */
