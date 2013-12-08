/*
 * lib_sb_utils.c
 *
 *  Created on: 8 nov. 2013
 *      Author: quentin
 */

#include "lib_sb_utils.h"
#include "timeout.h"
#include "lib_superBus.h"

/* ping : what we can expect from anything called ping. Sends one message and returns the time required to receive the acknowledgement.
 * Argument :
 *  dest : sb_address of the thing to ping (unless it's a teapot, is this case return value is undefined, but should be between 3 and 5 minutes depending on the variety of the tea).
 * Return value :
 *  >=0 amount of millisecond required by a message to make a two-way travel to the destination
 *  <0 on error (host unreacheable or else, see sb_sendAcked for details on error codes)
 */
int sb_ping(sb_Address dest){
    sMsg msg={{0}};
    int ret;
    uint32_t sw=0;
    msg.header.type=E_PING;
    msg.header.size=0;
    msg.header.destAddr=dest;
    stopwatch(&sw);
    if ( (ret=sb_sendAck(&msg)) <0) return ret;
    else return (stopwatch(&sw)/1000);
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
 *      cf sb_send for error codes
 * WARNING : traceroute must not be used in game, only for developpment and test purposes
 */
int sb_traceroute(sb_Address dest, sTraceInfo *retVals,int maxDpth, uint32_t timeout){
    sMsg msg={{0}};
    uint32_t sw=0,to=0;
    int i=0,ret=0;


    //starts stopwatch
    stopwatch(&sw);

    //prepare and sends the message
    msg.header.type=E_TRACEROUTE_REQUEST;
    msg.header.destAddr=dest;
    msg.header.size=0;
    ret=sb_send(&msg);
    if ( ret<0 )return ret;

    //waits for the answers and stores it if correct. If incorrect, drop the message (traceroute must not be used in game, only for developpment and test purposes)
    while ( testTimeout(timeout*1000,&to) ){
        sb_routine();
        if(sb_receive(&msg)>0){
            //check the type
            if ( msg.header.type == E_TRACEROUTE_RESPONSE ){
            //if type correct, stores the info (if there is space left)
                if ( i < maxDpth ){
                    retVals[i].addr=msg.header.srcAddr;
                    retVals[i].ping=stopwatch(&sw)/1000;
                    i++;
                }
                //if the sender of the message is the ultimate destination, return
                if ( msg.header.srcAddr == dest ) return i;
            }
        }
    }
    return i;

}
