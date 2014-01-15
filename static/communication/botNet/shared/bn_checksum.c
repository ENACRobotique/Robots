/*
 * lic_comm.cpp
 *
 *  Created on: 4 mai 2013
 *      Author: quentin
 */

#include "bn_checksum.h"
#include "../../../global_errors.h"


/*
Checksum algorithm :
    To compute :
        sum = sum of all the bytes of the message (header and payload), exluding the "checksum" field in the header
        sum&=0xff : Keep the last 8 bits (logical "AND" with 0xff)
        checksum field on the header : checksum=0xff-sum+1
    To test
        add all the bytes of the message, result should be 0.


 */

/* Verifies the integrity of the message using the checksum field in the header
 * Argument :
 *  msg :pointer to the memory area containing the header to check
 * Return value :
 *  0 if checksum correct, negative error code otherwise
 *
 * Remark : header must be without offset between his bytes
 */
uint8_t checkSum(sMsg *msg){
    int i;
    uint8_t sum=0;
    for ( i=0 ; i < sizeof(sGenericHeader)+msg->header.size ; i++ ){
        sum+=((uint8_t *)msg)[i];
    }
    if (sum) return -ERR_BN_CSUM;
    else return 0;
}

/* Sets the "checksum" field in the header of the mesage at msg.
 * Argument :
 *  msg : pointer to the memory area where the message to handle is located.
 * Return value :
 *  none.
 *
 * Remark : the message must be "ready to send", every other header feild must already be set,especially
 * the "size" field must be correctly set before calling setSum;
 */
void setSum(sMsg *msg){
    int i;
    uint8_t sum=0;
    for ( i=0 ; i < sizeof(sGenericHeader)+msg->header.size ; i++ ){
        sum+=((uint8_t *)msg)[i];
    }
    sum-=msg->header.checksum;
    msg->header.checksum = 0xff - sum + 1;
    return;
}
