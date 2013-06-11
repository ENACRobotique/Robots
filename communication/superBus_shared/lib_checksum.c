/*
 * lic_comm.cpp
 *
 *  Created on: 4 mai 2013
 *      Author: quentin
 */

#include "lib_checksum.h"

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
    for (i=0;i<sizeof(sGenericHeader)-1;i++){
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
uint8_t cbChecksumHead(uint8_t *pt, uint8_t size, uint8_t lastB){
    int i;
    uint8_t sum=0;
    for (i=1;i<sizeof(sGenericHeader);i++){
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
    for (i=0;i<sizeof(sGenericHeader)-2;i++){
        sum+=((uint8_t *)pt)[i];
    }
    return (~sum)+1;
}


/* Computes the checksum
 * Argument :
 *  msg : message of which payload must be checked
 * Return value :
 *  0 if checksum not correct, non-zero value otherwise
 *
 * Remark : header must be without offset between his bytes
 */
uint8_t checksumPload(sMsg *msg){
    int i;
    uint8_t sum=0;
    for (i=0;i<msg->header.size;i++){
        sum+=msg->payload.raw[i];
    }
    sum+=msg->header.checksumPload;
    return !sum;
}

/*
 * Compute the checksum to include in a generic header
 * Argument :
 *  size : size (in bytes) of the payload to checksum
 */
uint8_t calcSumPload(uPayload *pt, int size){
    int i;
    uint8_t sum=0;
    for (i=0;i<size;i++){
        sum+=pt->raw[i];
    }
    return (~sum)+1;
}

void setSum(sMsg *msg){
    msg->header.checksumHead=calcSumHead(&(msg->header));
    msg->header.checksumPload=calcSumPload(&(msg->payload),msg->header.size);
}
