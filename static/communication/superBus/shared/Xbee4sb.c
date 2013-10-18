/*
 * Xbee4sb.c
 *
 *  Created on: 28 juin 2013
 *      Author: quentin
 */

#include "Xbee_API.h"
#include "Xbee4sb.h"
#include "network_cfg.h"
#include "messages.h"
#include "node_cfg.h"
#include "lib_superBus.h"

#include <string.h>


/* setupXbee :
 *  Sets the parameters of the Xbee
 * Return value :
 *      1 if ok
 *      -1 if error
 *      -2 if config frame to statused
 *      -3 if error for wrong command/parameter or ERROR
 *
 */
int setupXbee(){
    int byteRead=0;
    spAPISpecificStruct stru;
    uint32_t sw=0;

    // TODO writes Xbee module config here (AT command)

    //writes node's address on the xbee
    XbeeATCmd("MY",12,XBEE_ATCMD_SET,MYADDRX);

    //waits for acknowledgement
    do {
        byteRead=XbeeReadFrame(&stru);
    } while( !(stru.APID==XBEE_APID_ATRESPONSE && stru.data.TXStatus.frameID==12) && testTimeout(SB_WAIT_XBEE_SND_FAIL,&sw));

    if (!byteRead || stru.APID!=XBEE_APID_ATRESPONSE || stru.data.TXStatus.frameID!=12) return -2;
    else if (stru.data.ATResponse.status!=0) return -3;


    //saves changes in non-volatile memory
    XbeeATCmd("WR",13,XBEE_ATCMD_SET,MYADDRX);
    do {
        byteRead=XbeeReadFrame(&stru);
    } while( !(stru.APID==XBEE_APID_ATRESPONSE && stru.data.TXStatus.frameID==13) );
    if (stru.data.ATResponse.status!=0) return -3;

    return 1;
}


void Xbee_init(){
    uint8_t garbage;

    //init the serial link
#ifdef ARCH_X86_LINUX
    serialInit(0,"/dev/ttyUSB0");
#elif defined(ARCH_328P_ARDUINO)
    serialInit(111111,0);
#else 
#error "no arch defined for Xbee4sb.c, or arch no available (yet)"
#endif

    //clear in buffer from remaining bytes
    while (serialRead(&garbage));

    //setup Xbee
    setupXbee();


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
    if ((size=XbeeReadFrame(&stru))<=0) return 0;

    //computes real size of payload
    size-=sizeof(stru.APID)+sizeof(stru.data.RX16Data.lSrcAddr_be)+sizeof(stru.data.RX16Data.options)+sizeof(stru.data.RX16Data.rssi);

    // if wrong type, return 0;
    if (stru.APID!=XBEE_APID_RX16) return 0;

    //oterwise (something red && good type), return size of frame red
    memcpy(pRet,(stru.data.RX16Data.payload),size);
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
 *      -1 if error on sending frame to Xbee
 *      -2 if error on receiving the status frame
 *      -3 if sending not successful (e.g. no ack)
 */
int Xbee_send(sMsg *msg, uint16_t nexthop){
    spAPISpecificStruct stru={0};
    uint32_t sw=0; //stopwatch
    int byteRead=0;

    if (!XbeeTx16(nexthop,0,37,msg,msg->header.size+sizeof(sGenericHeader))) return -1;

    do {
        byteRead=XbeeReadFrame(&stru);

        if ( byteRead && stru.APID==XBEE_APID_RX16){
            sb_pushInBufLast((sMsg*)&(stru.data.RX16Data.payload),IF_XBEE);
            byteRead=0;
        }
    } while( !(stru.APID==XBEE_APID_TXS && stru.data.TXStatus.frameID==37) && testTimeout(SB_WAIT_XBEE_SND_FAIL,&sw));

    if (!byteRead || stru.APID!=XBEE_APID_TXS || stru.data.TXStatus.frameID!=37) return -2;

    else {
        if (stru.data.TXStatus.status==XBEE_TX_S_SUCCESS) return (msg->header.size+sizeof(sGenericHeader));
        else return -3;
    }



}

