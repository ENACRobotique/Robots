/*
 * Xbee4bn.c
 *
 *  Created on: 28 juin 2013
 *      Author: quentin
 */

// config files
#include "node_cfg.h"
#include "network_cfg.h"
#include "messages.h"

#if MYADDRX
// other required libraries
#include "../../../tools/libraries/Timeout/timeout.h"
#include "../../Xbee_API/shared/Xbee_API.h"
#include "../../UART_framing/shared/lib_UART_framing.h"
#include "global_errors.h"

// superBus specific libraries
#include "Xbee4bn.h"
#include "botNet_core.h"

// std libs
#include <string.h>
#include <stdint.h>




/* setupXbee :
 *  Sets the parameters of the Xbee
 * Return value :
 *      0 if ok
 *      -1 if error
 *      -2 if config frame not statused
 *      -3 if error for wrong command/parameter or ERROR
 */
int Xbee_setup(){
    int frID=12;
    int ret;

    // /!\ you need to set BD=7 and AP=2 /!\ .

//writes CH parameter on the xbee (channel)
    if ( (ret=Xbee_ATCmd("CH",frID,XBEE_ATCMD_SET,0x0E))<0 ) return ret;

    if ( (ret=Xbee_waitATAck(frID,BN_WAIT_XBEE_SND_FAIL))<0 ) return ret;
    frID++;

//writes ID parameter on the xbee (PAN-id)
    if ( (ret=Xbee_ATCmd("ID",frID,XBEE_ATCMD_SET,0x34ac))<0 ) return ret;

    if ( (ret=Xbee_waitATAck(frID,BN_WAIT_XBEE_SND_FAIL))<0 ) return ret;
    frID++;

//writes node's address on the xbee
    if ( (ret=Xbee_ATCmd("MY",frID,XBEE_ATCMD_SET,MYADDRX))<0 ) return ret;

    if ( (ret=Xbee_waitATAck(frID,BN_WAIT_XBEE_SND_FAIL))<0 ) return ret;
    frID++;

//writes MM parameter on the xbee to match peer-to-peer use (mac mode)
    if ( (ret=Xbee_ATCmd("MM",frID,XBEE_ATCMD_SET,2))<0 ) return ret;

    if ( (ret=Xbee_waitATAck(frID,BN_WAIT_XBEE_SND_FAIL))<0 ) return ret;
    frID++;

//writes RN parameter on the xbee (random delay)
    if ( (ret=Xbee_ATCmd("RN",frID,XBEE_ATCMD_SET,0x01))<0 ) return ret;

    if ( (ret=Xbee_waitATAck(frID,BN_WAIT_XBEE_SND_FAIL))<0 ) return ret;
    frID++;

//writes CE parameter on the xbee to match peer-to-peer use
    if ( (ret=Xbee_ATCmd("CE",frID,XBEE_ATCMD_SET,0))<0 ) return ret;

    if ( (ret=Xbee_waitATAck(frID,BN_WAIT_XBEE_SND_FAIL))<0 ) return ret;
    frID++;

//writes A1 parameter on the xbee to match peer-to-peer use
    if ( (ret=Xbee_ATCmd("A1",frID,XBEE_ATCMD_SET,0))<0 ) return ret;

    if ( (ret=Xbee_waitATAck(frID,BN_WAIT_XBEE_SND_FAIL))<0 ) return ret;
    frID++;



//writes RN parameter on the xbee to enable collision avoidance on first iteration of CSMA/CA
    if ( (ret=Xbee_ATCmd("RN",frID,XBEE_ATCMD_SET,1))<0 ) return ret;

    if ( (ret=Xbee_waitATAck(frID,BN_WAIT_XBEE_SND_FAIL))<0 ) return ret;
    frID++;

    //saves changes in non-volatile memory
    if ( (ret=Xbee_ATCmd("WR",frID,XBEE_ATCMD_SET,MYADDRX))<0 ) return ret;

    if ( (ret=Xbee_waitATAck(frID,BN_WAIT_XBEE_SND_FAIL*4L))<0 ) return ret;
    frID++;

    return 0;
}



/* handle the reading of the data from the Xbee on the serial port.
 * return value : nb of bytes written in pRet, 0 on error or non-detection of start sequence
 * pointer return : message in a structure
 * requires : Serial initialisation
 *
 * Remark : after a call to Xbee_receive, the memory area designated by pRet may be modified even if no valid message was received
 */
int Xbee_receive(sMsg *pRet){
    spAPISpecificStruct stru;
    int size=0;
    //read one frame. If nothing red, return 0
    if ((size=Xbee_readFrame(&stru))<=0) return size;

    //computes real size of payload
    size-=sizeof(stru.APID)+sizeof(stru.data.RX16Data.lSrcAddr_be)+sizeof(stru.data.RX16Data.options)+sizeof(stru.data.RX16Data.rssi);

    // if wrong type, return 0;
    if (stru.APID!=XBEE_APID_RX16) return 0;

    //oterwise (something red && good type), return size of frame red
    memcpy(pRet,(stru.data.RX16Data.payload),MIN(size,100));
    return size;
}

/*
 * Handle the sending of the message to the Xbee via the serial.
 * Size & checksum of msg must be set before calling Xbee_send
 * Waits until the sending request was acknowledged (statused), or untils timeout
 *
 * Argument :
 *  msg : message to send (thanks captain obvious!)
 *  nextHop : adress of the nex hop for this message
 * Return value :
 *  number of bytes of the message send
 *  <0 if error :
 *      error on sending frame to Xbee
 *      error on receiving the status frame
 *      sending not successful (e.g. no ack)
 */
int Xbee_send(const sMsg *msg, uint16_t nexthop){
    spAPISpecificStruct stru={0};
    uint32_t sw=0; //stopwatch
    int byteRead=0;
    int ret=0;

    if ( (ret=Xbee_Tx16(nexthop,0,37,msg,msg->header.size+sizeof(sGenericHeader)))<=0 ) return ret;

    do {
        byteRead=Xbee_readFrame(&stru);

        if ( byteRead>0 && stru.APID==XBEE_APID_RX16){
            bn_pushInBufLast((sMsg*)&(stru.data.RX16Data.payload),IF_XBEE);
            byteRead=0;
        }
        else if(byteRead < 0){
            return byteRead;
        }
    } while( !(byteRead>0 && stru.APID==XBEE_APID_TXS && stru.data.TXStatus.frameID==37) && testTimeout(BN_WAIT_XBEE_SND_FAIL,&sw));

    if (!byteRead || stru.APID!=XBEE_APID_TXS || stru.data.TXStatus.frameID!=37) return -ERR_XBEE_NOSTAT;

    else {
        if (stru.data.TXStatus.status==XBEE_TX_S_SUCCESS) return (msg->header.size+sizeof(sGenericHeader));
        else return -ERR_XBEE_NOACK;
    }
    // unreachable
}

#endif //MYADDRX
