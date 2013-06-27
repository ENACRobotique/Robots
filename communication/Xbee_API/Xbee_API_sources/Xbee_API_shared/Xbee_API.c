/*
 * Xbee_API.c
 *
 *  Created on: 14 juin 2013
 *      Author: quentin
 *
 *      This library was created to use XBee devices with API mode 2 (with escaped characters).
 *      Though some libraries already exists for Arduino, they uses a lot of C++ functionalities and
 *      Arduino-specific funtions.
 *      In this one, I tried to be as architecture-independant as possible, and I used only C language.
 *      In consequence, this library can be used on any device, which can be programmed in C (microcontroller,
 *      PC...) and owns a serial (UART) link.
 *
 *      Because of this portability, there are a few things to do if you want to use it :
 *          1)  Your project must have a "params.h" file, visible from Xbee_API.h, which contains :
 *              #define ARCH_XXX                           // According to the #ifdef at the beginning of XBee_API.c
 *              #define LITTLE_ENDIAN or BIG_ENDIAN
 *              #define XBEE_WAITFRAME_TIMEOUT  1000000    // in microsecond
 *              #define XBEE_READBYTE_TIMEOUT   5000       // in microsecond
 *          2)  If needed, you will have to write some "serial drivers". These are  Xbee_API_XXX_drivers.h and
 *              Xbee_API_XXX_drivers.c files in "drivers" folder, implementing the functions described in
 *              "defaultSerialForXbee.h"
 *          3)  Finally, you will have to modify "Xbee_API.c" to match your files. The simplest is to add :
 *                  "
 *                  #ifdef ARCH_XXX                             // the one of param.h
 *                  #include "drivers/Xbee_API_XXX_drivers.h"   //the one you juste wrote
 *                  "
 *                  Then replace the first #ifdef ARCH_YYY of the previous list by #elif defined(ARCH_YYY)
 *
 *
 */

#include "Xbee_API.h"
#include "params.h"

#include <string.h>




#if (defined(BIG_ENDIAN) && defined(LITTLE_ENDIAN))
#error bi-endianess or esle not supported
#elif (!defined(BIG_ENDIAN) && !defined(LITTLE_ENDIAN))
#error endianess MUST be defined
#endif

/* Xbee TX request for 16bits addresses
 * Arguments :
 *  to_h : address of the destination Xbee, Host endianess
 *  options :  XBEE_TX_O_BCAST xor XBEE_TX_O_NOACK
 *  frameID : for ack purposes (not used if NOACK or BCAST).
 *  data : pointer to the data to send over Xbee.
 *  datasize : size that should be read at data.
 * Return value : 0 if error, nb of bytes written on serial link (including start, size and checksum, but exluding escaping char)
 *
 */
int XbeeTx16(XbeeAddr16_t to_h,uint8_t options, uint8_t frameID, const void* data, uint16_t dataSize){
    spAPISpecificStruct stru={0};

    //writes API cmd ID
    stru.APID=XBEE_APID_TX16;

    //Writes destination  and frameID according to option selected
    if (options&XBEE_TX_O_BCAST){
        stru.data.TX16Data.lDstAddr_be=0xffff;
        stru.data.TX16Data.options=XBEE_TX_O_BCAST;
        stru.data.TX16Data.frameID=0;
    }
    else {
        stru.data.TX16Data.options=options;

        if (options==XBEE_TX_O_NOACK) stru.data.TX16Data.frameID=0;
        else stru.data.TX16Data.frameID=frameID;

        //endianess-proof address writing
        stru.data.TX16Data.lDstAddr_be=hbe2_swap(to_h);
    }

    //payload writing : optimization possible, play with pointer to avoid useless copy
    //but will require to have a smart XbeeWriteFrame (or sending done here)
    memcpy(stru.data.TX16Data.pPayload,data,dataSize);


    return XbeeWriteFrame(&stru,dataSize+5);
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
 *  parameter_h : parameter (may be optional). Ignored if XBEE_ATCMD_GET
 * Return value : nb of bytes written on serial link (including start, size and checksum, but exluding escaping char)
 *
 * Remark : does not perform check on size and consistency of parameter
 * /!\ refer to XBEE doc for available commands
 */
int XbeeATCmd(char cmd[2],uint8_t frameID, uint8_t option, uint32_t parameter_h){
    spAPISpecificStruct sCmd;

    // sets API specific ID
    sCmd.APID=XBEE_APID_ATCMD;

    //0 means no answer
    sCmd.data.ATCmd.frameID=frameID;

    // Writes the command
    sCmd.data.ATCmd.cmd[0]=cmd[0];
    sCmd.data.ATCmd.cmd[1]=cmd[1];

    if (option == XBEE_ATCMD_SET){
        // converts the parameter to big endian
        sCmd.data.ATCmd.parameter_be=hbe4_swap(parameter_h);

        return XbeeWriteFrame(&sCmd,8);
    }
    else return XbeeWriteFrame(&sCmd,4);

    //TODO : add wait until satuts
}

/* XbeeWriteFrame : writes a specific frame one the serial link, for the Xbee.
 * Return value : nb of bytes written on the serial link (Excluding escaping caracter 0x7D), or 0 if error
 * parameters :
 *  size : size of data to send (memory area pointed to by str_be), INCLUDING API command identifier
 *  str_be : api-specific structure to send (/!\ data to be understood by Xbee module MUST be big-endian, unspecified for the rest)
 */
int XbeeWriteFrame(const spAPISpecificStruct *str_be, uint16_t size_h){
    uint8_t checksum=0;
    int count=0;

    // write start byte
    if (!serialWrite(XBEE_FRAME_START)) return 0;

    // write size in the right order (big-endian)
    if (!XbeeWriteByteEscaped((uint8_t)(size_h>>8))) return 0;
    if (!XbeeWriteByteEscaped((uint8_t)(size_h&0xff))) return 0;

    // writes the API command ID
    if (!XbeeWriteByteEscaped(str_be->APID)) return 0;
    checksum+=str_be->APID;

    // writes the rest of the frame and compute checksum
    while (count!=(size_h-1)){
        checksum+=str_be->data.raw[count];
        if (!XbeeWriteByteEscaped(str_be->data.raw[count])) return 0;
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
 *  size of the frame written in *frame if correct,
 *  0 if no frame available after frame timeout
 *  -1 if bad checksum
 *  -2 if byte timeout (but start character detected)
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
    if (readByte!=XBEE_FRAME_START) {
        return 0;
    }
    testTimeout(0); //resets the timeout

    //reading size of message (with timeout)
    while (!(lus=XbeeReadByteEscaped(&readByte)) && testTimeout(XBEE_READBYTE_TIMEOUT));
    if (!lus) return -2;
    testTimeout(0);
    while (!(lus=XbeeReadByteEscaped(&readByte1)) && testTimeout(XBEE_READBYTE_TIMEOUT));
    if (!lus) return -2;
    testTimeout(0);

    size= (readByte<<8) | readByte1 ; //endianness-proof

    //read size bytes
    while (count != size){
        while (!(lus=XbeeReadByteEscaped(&rawFrame[count])) && testTimeout(XBEE_READBYTE_TIMEOUT));
        if (!lus) return -2;
        testTimeout(0);
        checksum+=rawFrame[count];
        count++;
    }

    //checksum (read byte ,add , test and return)
    while (!(lus=XbeeReadByteEscaped(&readByte)) && testTimeout(XBEE_READBYTE_TIMEOUT));
    if (!lus) return -2;
    testTimeout(0);
    checksum+=readByte;

    if (checksum == 0xff) return count;
    return -1;
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















