/*
 * utils.c
 *
 *  Created on: 8 nov. 2013
 *      Author: quentin
 */

#include "bn_utils.h"
#include "../../tools/libraries/Timeout/timeout.h"
#include "global_errors.h"

/* ping : what we can expect from anything called ping. Sends one message and returns the time required to receive the acknowledgement.
 * Argument :
 *  dest : bn_address of the thing to ping (unless it's a teapot, is this case return value is undefined, but should be between 3 and 5 minutes depending on the variety of the tea).
 * Return value :
 *  >=0 amount of millisecond required by a message to make a two-way travel to the destination
 *  <0 on error (host unreacheable or else, see error codes)
 */
int bn_ping(bn_Address dest){
    sMsg msg={{0}};
    int ret;
    uint32_t sw=0;
    msg.header.type=E_PING;
    msg.header.size=0;
    msg.header.destAddr=dest;
    stopwatch(&sw);
    if ( (ret=bn_sendAck(&msg)) <0) return ret;
    else return (stopwatch(&sw)/1000);
}


/* pingLink : Sends one message in linkcast and returns the time required to receive the acknowledgement for each reply. Will wait for
 * answers during waitingDelay.
 * Argument :
 *  dest : bn_address of the thing to ping (unless it's a teapot, is this case return value is undefined, but should be between 3 and 5 minutes depending on the variety of the tea).
 *  waitingDelay : duration in ms during which we will wait for replies.
 *  retArray : pointer to an array of sTraceIfo which will store the results of the ping.
 *  sizeRet : size of retArray. Only the first retArray will be stored, the rest will be dropped.
 * Return value :
 *  >=0 number of answers received during waitingDelay. May exceed sizeRet, user should check this to detect dropped messages.
 *  <0 on error (see error codes)
 *
 * XXX source of sequence loss. Do not use in game
 */
int bn_pingLink(bn_Address dest, uint32_t waitingDelay, sTraceInfo retArray[], int retSize){
    sMsg msg={{0}};
    int ret;
    uint32_t sw=0;
    int index=0;
    int receiveVal=0;
    uint8_t tmpSeqNum=seqNum;
    uint8_t ackValue=0;

    if (bn_isLinkcast(dest)){
        msg.header.type=E_PING;
        msg.header.size=0;
        msg.header.destAddr=dest;
        msg.header.ack=1;

        stopwatch(&sw);
        if ( (ret=bn_genericSend(&msg)) <0) return ret;
        while (stopwatch(&sw)/1000 <= waitingDelay){
            // read incoming
            if ( (receiveVal=bn_receive(&msg)) > 0 ) {
                //if this message is not an ack response,  put it back in the incoming buffer (sent to self)
                if ( msg.header.type != E_ACK_RESPONSE ){
                    bn_pushInBufLast(&msg,IF_LOCAL);
                }
                // if this msg is not the ack expected (not the good destination or seqnum), drop it (do nothing)
                else if ( msg.payload.ack.addr != dest || msg.payload.ack.seqNum != tmpSeqNum) continue;
                // if this message is an ack , from the requested destination and with the correct seqnum
                else {
                    // increments the number of recorded acks
                    index++;
                    // store details if free space in array
                    if (index < retSize){
                        retArray[index].addr = msg.header.srcAddr;
                        retArray[index].ping = stopwatch(&sw);
                        if ( msg.payload.ack.ans == A_ACK) {
                            retArray[index].error = 0;
                        }
                        else if ( msg.payload.ack.ans == A_NACK_BROKEN_LINK) {
                            retArray[index].error = -ERR_BN_NACK_BROKEN_LINK;
                        }
                        else if ( msg.payload.ack.ans == A_NACK_BUFFER_FULL) {
                            retArray[index].error = -ERR_BN_NACK_FULL_BUFFER; //XXX behavior?
                        }
                    }
                }
            }
            else if (receiveVal < 0) {
                return receiveVal;
            }
        }
        return index;
    }
    else return -ERR_BN_NO_LCAST_ADDR;
}


/* traceroute : simple traceroute utility.
 * Arguments :
 *  dest : should I really write something here ?
 *  *retvals : pointer to a table of sTraceInfo of L elements.
 *  macDpth : maximum number of relay to display (should in fact be L). The macDpth+1 node on the path will not be displayed.
 *  timeout : time after which we consider the host as unreachable (in ms).
 * Return value :
 *  if >0 : number of elements correctly written in retVals (ie nb of nodes on the way to dest, including dest, or L if there are too many nodes)
 *  if <0 : error !
 *      cf error codes
 * WARNING : traceroute must not be used in game, only for developpment and test purposes
 */
int bn_traceroute(bn_Address dest, sTraceInfo *retVals,int maxDpth, uint32_t timeout){
    sMsg msg={{0}};
    uint32_t sw=0,to=0;
    int i=0,ret=0;



    //prepare  the message
    msg.header.type=E_TRACEROUTE_REQUEST;
    msg.header.destAddr=dest;
    msg.header.size=0;

    //starts stopwatch
    stopwatch(&sw);

    //sends the message
    ret=bn_send(&msg);

    if ( ret<0 )return ret;

    //waits for the answers and stores it if correct. If incorrect, drop the message (traceroute must not be used in game, only for development and test purposes)
    while ( testTimeout(timeout*1000,&to) ){
        if(bn_receive(&msg)>0){
            //check the type
            if ( msg.header.type == E_TRACEROUTE_RESPONSE ){
            //if type correct, stores the info (if there is space left)
                if ( i < maxDpth ){
                    retVals[i].addr=msg.header.srcAddr;
                    retVals[i].ping=stopwatch(&sw)/1000;
                    retVals[i].error = 0;
                    i++;
                }
                //if the sender of the message is the ultimate destination, return
                if ( msg.header.srcAddr == dest ) return i;
            }
        }
    }
    return i;

}
