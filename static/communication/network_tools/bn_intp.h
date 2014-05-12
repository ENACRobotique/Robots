/*
 * bn_intp.h : botnet's Network Time Protocol Inverted (master-slave instead of client-server in plain NTP)
 *
 *  Created on: 2 mai 2014
 *      Author: quentin
 */

#ifndef BN_INTP_H_
#define BN_INTP_H_



#include "messages.h"
#include "../botNet/shared/botNet_core.h"

#ifdef __cplusplus
extern "C" {
#endif

extern uint32_t bn_intp_MicrosOffset;
extern uint32_t bn_intp_minDT;

/* bn_intpMsgHandle : handles synchronization messages from master.
 * MUST be bn_attached on every slave to messages of type E_INTP (done in bn_intp_install)
 *
 */
void bn_intp_msgHandle(sMsg *msg);

/* bn_intp_install : install the intp slave handle
 * MUST be called on every slave.
 */
static inline int bn_intp_install(){
    return bn_attach(E_INTP,bn_intp_msgHandle);
}

/* bn_intp_micros2s : converts a local date in microsecond to a synchronized date. returned value are valid only after a synchronization has been done
 * Argument :
 *  micros : date to convert
 * Return value : converted date
 */
static inline uint32_t bn_intp_micros2s(uint32_t micros){
    return micros-bn_intp_MicrosOffset;
}

/* bn_intp_isSync : tell if synchronization has been done.
 * Return value :
 *  0 if unsynchronized
 *  anything else if synchronized
 */
static inline int bn_intp_isSync(){
    return bn_intp_minDT;
}

/* bn_intp_sync : Makes the synchronization with device at address slave.
 * Argument :
 *  slave : address of the device to synchronize (must have bn_intp installed, cf bn_intp_install)
 * Return value :
 *  >0 if synchronization has succeed (amount of pair of synchronization messages acked).
 *  <0 otherwise (see global_errors.h).
 */
int bn_intp_sync(bn_Address slave, int retries);


#ifdef __cplusplus
}
#endif


#endif /* BN_INTP_H_ */
