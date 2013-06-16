/*
 * Xbee_API.c
 *
 *  Created on: 14 juin 2013
 *      Author: quentin
 */

#include "Xbee_API.h"
#include "params.h"


#ifdef ARCH_X86_LINUX
#include "Xbee_API_linux_drivers.h"
#include <stdio.h>
#else
#error will not compile, check architecture define and driver library
#endif


/* Xbee TX request for 16bits addresses
 * Return value : nb of bytes written to serial port, 0 if error
 */
int XbeeTx16(XbeeAddr16_t to,uint8_t options, uint8_t frameID, void* data, uint16_t datasize){
    return 0;
}

/* Xbee TX status reading
 * Return value : nb of bytes red from serial port, 0 if error
 * status written in *status
 */
int XbeeGetTxStatus(spTXStatus *status){
    return 0;
}

/* XbeeATCmd : sends an AT command
 * Arguments :
 *  cmd : string naming the command
 *  frameID : for acknowledgment purposes. 0 means no acknowledgment.
 *  option : XBEE_ATCMD_SET xor XBEE_ATCMD_GET.
 *  parameters : parameter (may be optional). Ignored if XBEE_ATCMD_GET
 * Return value : TODO
 *
 * Remark : does not perform check on size and consistency of parameter
 * /!\ refer to XBEE doc for available commands
 */
int XbeeATCmd(char cmd[2],uint8_t frameID, uint8_t option, uint32_t parameter){
    int i;
    spAPISpecificStruct sCmd;

    // sets API specific ID
    sCmd.APID=XBEE_APID_ATCMD;

    //0 means no answer
    sCmd.data.ATCmd.frameID=frameID;

    // converts the command to big endian
    sCmd.data.ATCmd.cmd[0]=cmd[0];
    sCmd.data.ATCmd.cmd[1]=cmd[1];

    if (option == XBEE_ATCMD_SET){
        // converts the parameter to big endian
        for (i=0;i<4;i++){
            ((uint8_t*)(&sCmd.data.ATCmd.parameter_be))[i]=(parameter>>(8*(3-i)))&0xff;
        }
        XbeeWriteFrame(8,sCmd);
    }
    else XbeeWriteFrame(4,sCmd);

    return 0;
}

/* XbeeWriteFrame : writes a specific frame one the serial link, for the Xbee.
 * Return value : nb of bytes actually written on the serial link, or 0 if error
 * parameters :
 *  size : size of data to send (memory area pointed to by data_be), INCLUDING API command identifier
 *  str_be : api-specific structure to send (/!\ data to be understood by Xbee module MUST be big-endian, unspecified for the rest)
 */
int XbeeWriteFrame(uint16_t size, spAPISpecificStruct str_be){
    uint8_t checksum=0;
    int count=0;

    // write start byte
    if (!serialWrite(XBEE_FRAME_START)) return 0;

    // write size in the right order (big-endian)
    if (!XbeeWriteByteEscaped((uint8_t)(size>>8))) return 0;
    if (!XbeeWriteByteEscaped((uint8_t)(size&0xff))) return 0;

    // writes the API command ID
    if (!XbeeWriteByteEscaped(str_be.APID)) return 0;
    checksum+=str_be.APID;

    // writes the rest of the frame and compute checksum
    while (count!=(size-1)){
        if (!XbeeWriteByteEscaped(str_be.data.raw[count])) return 0;
        checksum+=str_be.data.raw[count];
        count++;
    }
    // final compute and write the checksum
    if (!XbeeWriteByteEscaped(0xff-checksum)) return 0;

    return (count+4);
}



/* XbeeReadFrame : reads a frame on the serial link, escapes the characters that have to bo escaped,
 * perform the checksum and if everything is correct return the number of bytes of the Frame Data correctly red.
 * Arguments :
 *  *frame : pointer to the memory area in which the frame shall be written
 * Return value :
 *  size of the frame written in *frame, 0 if no frame available after timeout or bad checksum
 */
int XbeeReadFrame(spAPISpecificStruct *str){
    uint8_t *rawFrame=(uint8_t*)str;
    uint16_t size=0;
    int count=0;
    uint8_t checksum=0;
    uint8_t readByte=0,readByte1=0;
    int lus;

    //waiting for a frame start byte
    while (readByte!=XBEE_FRAME_START && testTimeout(XBEE_WAITFRAME_TIMEOUT)){
        lus=serialRead(&readByte);
    }
    if (readByte!=XBEE_FRAME_START) {//FIXME !lus) {
        printf("timeout readFrame\n");
        return 0;
    }
    testTimeout(0); //restes the timeout

    //reading size of message (with timeout)
    while (!(lus=XbeeReadByteEscaped(&readByte)) && testTimeout(XBEE_READBYTE_TIMEOUT));
    if (!lus) return 0;
    testTimeout(0);
    while (!(lus=XbeeReadByteEscaped(&readByte1)) && testTimeout(XBEE_READBYTE_TIMEOUT));
    if (!lus) return 0;
    testTimeout(0);

    size= (readByte<<8) | readByte1 ; //endianness-proof

    //read size bytes
    while (count != size){
        while (!(lus=XbeeReadByteEscaped(&rawFrame[count])) && testTimeout(XBEE_READBYTE_TIMEOUT));
        if (!lus) return 0;
        testTimeout(0);
        checksum+=rawFrame[count];
        count++;
    }

    //checksum (read byte ,add , test and return)
    while (!(lus=XbeeReadByteEscaped(&readByte)) && testTimeout(XBEE_READBYTE_TIMEOUT));
    if (!lus) return 0;
    testTimeout(0);
    checksum+=readByte;

    if (checksum == 0xff) return count;
    return 0;
}


/* XbeeWriteByteEscaped : writes `byte` on the serial link, including escaping if needed
 * Return value :
 *  1 if write succeed, 0 otherwise
 */
int XbeeWriteByteEscaped(uint8_t byte){
    if ( byte == XBEE_FRAME_START || byte == XBEE_XOFF || byte == XBEE_XON || byte == XBEE_ESCAPE_CHAR ){
        if (serialWrite(XBEE_ESCAPE_CHAR) && serialWrite( byte^XBEE_ESCAPE_MASK )) return 1;
        return 0;
    }
    else if (serialWrite(byte)) return 1;
    return 0;
}

/* XbeeWriteByteEscaped : writes one byte on the serial link, removing escaping if needed
 * Return value :
 *  1 if read succeed, 0 otherwise
 */
int XbeeReadByteEscaped(uint8_t *byte){
    if(serialRead(byte)){
        // un-escape characters if needed
        if (*byte == XBEE_ESCAPE_CHAR){
            if (!serialRead(byte)) return 0;
            (*byte)^=XBEE_ESCAPE_MASK;
        }
        return 1;
    }
    return 0;
}















