/*
 * roles.h
 *
 *  Created on: 5 mars 2014
 *      Author: ludo6431
 */

#ifndef ROLES_H_
#define ROLES_H_

#include <stdint.h>

#include "node_cfg.h"
#include "messages.h"

// Lightweight system allowing one node (ex. gtk_monitoring_hmi) to configure on the fly
// all the other predefined nodes for message forwarding and relaying.
// 5 nodes are currently defined with specific roles (MONITORING,AI,PROPULSION,VIDEO,DEBUG)
// without rebuilding any device, a user can change the logic topology of the network
//
// ex1: ai & monitoring on a pc ; prop on the robot
// ex2: ai & prop on the robot ; monitoring on a pc
// those two examples needs different address configurations to do their job,
//      a role_setup message sent from monitoring_hmi to each node will setup the addresses
//
// a role_setup message may also be used to declare yourself as the new debug sink for each node

// interprets E_ROLE_SETUP messages, suitable for bn_attach
void        role_setup          (sMsg *msg);
// sets address used on this node to talk to the associated role
int         role_set_addr       (uint8_t role, bn_Address address);
// gives address currently associated with this role
bn_Address  role_get_addr       (uint8_t role);
// gives role associated with any ADDRESS on the botNet network
uint8_t     role_get_role       (bn_Address address);
const char *role_string(uint8_t role);
// sends data according to the rules defined during setup
int         role_send           (sMsg *msg, eRoleMsgClass mc);
int         role_sendAck        (sMsg *msg, eRoleMsgClass mc);
int         role_sendRetry      (sMsg *msg, eRoleMsgClass mc, int retries);
#if MYROLE
// relays data according to the rules defined during setup
int         role_relay          (sMsg *msg);
#endif

#endif /* ROLES_H_ */
