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
 *              #define ARCH_LITTLE_ENDIAN or ARCH_BIG_ENDIAN
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


// config files
#include "Xbee_API.h"

// other required libraries
#include "../../UART_framing/shared/lib_UART_framing.h"
#include "../../../global_errors.h"
#include "../../../tools/libraries/Timeout/timeout.h"

// standard libraries
#include <string.h>
#include <stdint.h>

#ifndef MIN
#define MIN(m, n) (m)>(n)?(n):(m)
#endif


#if (defined(ARCH_BIG_ENDIAN) && defined(ARCH_LITTLE_ENDIAN))
#error bi-endianess or esle not supported
#elif (!defined(ARCH_BIG_ENDIAN) && !defined(ARCH_LITTLE_ENDIAN))
#error endianess MUST be defined
#endif

/* Xbee TX request for 16bits addresses
 * Arguments :
 *  to_h : address of the destination Xbee, Host endianess
 *  options :  XBEE_TX_O_BCAST xor XBEE_TX_O_NOACK
 *  frameID : for ack purposes (not used if NOACK or BCAST).
 *  data : pointer to the data to send over Xbee.
 *  datasize : size that should be read at data.
 * Return value : cf Xbee_writeFrame
 *
 */
int Xbee_Tx16(XbeeAddr16_t to_h,uint8_t options, uint8_t frameID, const void* data, uint16_t dataSize){
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
    memcpy(stru.data.TX16Data.pPayload,data,MIN(dataSize,100));


    return Xbee_writeFrame(&stru,dataSize+5);
}


/* XbeeATCmd : sends an AT command
 * Arguments :
 *  cmd : string naming the command
 *  frameID : for acknowledgment purposes. 0 means no acknowledgment asked.
 *  option : XBEE_ATCMD_SET xor XBEE_ATCMD_GET.
 *  parameter_h : parameter (may be optional). Ignored if XBEE_ATCMD_GET
 * Return value : nb of bytes written on serial link (including start, size and checksum, but excluding escaping char)
 *
 * Remark : does not perform check on size and consistency of parameter
 * /!\ refer to XBEE doc for available commands
 * TODO : everything in one function (send cmd, waits status frame, return value)
 */
int Xbee_ATCmd(char cmd[2],uint8_t frameID, uint8_t option, uint32_t parameter_h){
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

        return Xbee_writeFrame(&sCmd,8);
    }
    else return Xbee_writeFrame(&sCmd,4);

}

/* Xbee_ATCmd : waits for the ack after an AT cmd.
 * Arguments :
 *  frID : for acknowledgment purposes. 0 means no acknowledgment asked.
 *  option : XBEE_ATCMD_SET xor XBEE_ATCMD_GET.
 * Return value : nb of bytes written on serial link (including start, size and checksum, but excluding escaping char)
 * Remark : does not perform check on size and consistency of parameter
 * refer to XBEE doc for available commands
 * /!\ do not use if returned values are expected !
 */
int Xbee_waitATAck(int frID, uint32_t timeOut){
    int byteRead=0;
    spAPISpecificStruct stru;
    uint32_t sw=0;

    //waits for acknowledgement
    do {
        byteRead=Xbee_readFrame(&stru);
    } while( !(byteRead>0 && stru.APID==XBEE_APID_ATRESPONSE && stru.data.TXStatus.frameID==frID) && testTimeout(BN_WAIT_XBEE_SND_FAIL,&sw));

    if (byteRead<=0 || stru.APID!=XBEE_APID_ATRESPONSE || stru.data.TXStatus.frameID!=frID) return -ERR_XBEE_NOSTAT;
    else if (stru.data.ATResponse.status==XBEE_ATR_S_ERROR) return -ERR_XBEE_AT_ERR;
    else if (stru.data.ATResponse.status==XBEE_ATR_S_INVCOM) return -ERR_XBEE_AT_WRONG_CMD;
    else if (stru.data.ATResponse.status==XBEE_ATR_S_INVPAR) return -ERR_XBEE_AT_WRONG_PAR;
    return 1;
}

/* XbeeWriteFrame : writes a specific frame one the serial link, for the Xbee.
 * Return value : nb of bytes written on the serial link (Excluding escaping caracter 0x7D), or 0 if error
 * parameters :
 *  size_h : size of data to send (memory area pointed to by str_be), INCLUDING API command identifier
 *  str_be : api-specific structure to send (/!\ data to be understood by Xbee module MUST be big-endian, unspecified for the rest)
 */
inline int Xbee_writeFrame(const spAPISpecificStruct *str_be, uint16_t size_h){
    return UART_writeFrame(str_be,size_h);
}



/* XbeeReadFrame : reads a frame on the serial link, escapes the characters that have to bo escaped,
 * perform the checksum and if everything is correct return the number of bytes of the Frame Data correctly red.
 * Arguments :
 *  *frame : pointer to the memory area in which the frame shall be written
 * Return value :
 *  >0 : size of the frame written in *frame if correct,
 *  <0 if error (check return for error code)
 */
inline int Xbee_readFrame(spAPISpecificStruct *str){
    return UART_readFrame(str,sizeof(spAPISpecificStruct));
}


int Xbee_init(){
    uint8_t garbage;
    int ret;
    //init the serial link
#ifdef ARCH_X86_LINUX
    ret=UART_init(XBEE_UART_PATH,E_115200_8N2);
#elif defined(ARCH_328P_ARDUINO)
    Xbee_rst();
    ret=UART_init(0,111111);
#else
#error "no arch defined for Xbee4sb.c, or arch no available (yet)"
#endif

    //waits for the Xbee to totally start
    uint32_t sw=0;
    while( testTimeout(10000000,&sw));

    //clear in buffer from remaining bytes
    while ( serialReadEscaped(&garbage,1000)>0 );
    if (ret<0) return ret;
    else return 1;
}













