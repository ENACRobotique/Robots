/*
 * lic_comm.cpp
 *
 *  Created on: 4 mai 2013
 *      Author: quentin
 */

#include "stdint.h"
#include "lib_checksum.h"

#include "messages.h"


/* Computes the checksum
 * Argument :
 *  pt :pointer to the memory area containing the header to check
 * Return value :
 *  0 if checksum not correct, non-zero value otherwise
 *
 * Remark : header must be without offset between his bytes
 */
uint8_t checksumHead(sGenericHeader *pt){
    int i;
    uint8_t sum=0;
    for (i=0;i<sizeof(sGenericHeader)-2;i++){
        sum+=((uint8_t *)pt)[i];
    }
    return !(sum);
}


/* Computes the checksum for the circulars buffers
 * Arguments :
 *  pt : pointer to the memory area containing the header to check
 *  size : size of the circular buffer (MUST be a power of 2)
 *  last : index of the byte in the rolling buffer corresponding to the last byte of the header
 * Return value :
 *  0 if checksum not correct, non-zero value otherwise
 *
 * Remarks :
 *  header must be  without offset between his bytes
 *  /!\ size MUST be a power of 2
 */
uint8_t cbChecksumHead(uint8_t *pt,uint8_t size, uint8_t lastB){
    int i;
    uint8_t sum=0;
    for (i=0;i<sizeof(sGenericHeader)-2;i++){
        sum+=pt[(lastB-i)&(size-1) ];
    }
    return !(sum);
}



/*
 * compute the checksum to include in a generic header
 *
 * remark : this checksum is also used as a "start" character than a real checksum. We rely on the xbee layer 2 to avoid data corruption
 */
uint8_t calcSumHead(sGenericHeader *pt){
    int i;
    uint8_t sum=0;
    for (i=0;i<sizeof(sGenericHeader)-3;i++){
        sum+=((uint8_t *)pt)[i];
    }
    return ~sum;
}


/* Computes the checksum
 * Argument :
 *  msg : message of which payload must be checked
 * Return value :
 *  0 if checksum not correct, non-zero value otherwise
 *
 * Remark : header must be without offset between his bytes
 */
uint8_t cbChecksumPload(sMsg msg){
    int i;
    uint8_t sum=0;
    for (i=0;i<msg.header.size;i++){
        sum+=((uint8_t *)&(msg.payload))[i];
    }
    return !(msg.header.checksumPload+sum);
}

/*
 * Compute the checksum to include in a generic header
 * Argument :
 *  size : size (in bytes) of the payload to checksum
 */
uint8_t calcSumPload(uPayload *pt,int size){
    int i;
    uint8_t sum=0;
    for (i=0;i<size;i++){
        sum+=((uint8_t *)pt)[i];
    }
    return ~sum;
}

void setSum(sMsg *msg){
    msg->header.checksumHead=calcSumHead(&(msg->header));
    msg->header.checksumPload=calcSumPload(&(msg->payload),msg->header.size);
}
