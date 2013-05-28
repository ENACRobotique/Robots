/*
 * lib_xbee.cpp
 *
 *  Created on: 5 mai 2013
 *      Author: quentin
 */

#include "messages.h"
#include "lib_checksum.h"
#include "Arduino.h"

//#define DEBUG_XBEE

void setupXbee(){
    //flush
    delay(1000);
//    while (Serial.available()){ //flush
//        Serial.read();
//    }
    Serial.print("+++");
    delay(1000);
    //network cfg
    //each node is an end device, with end device association disabled (factory config)
    //Serial.print("ATCE0,A10");


    //networkID
    Serial.println("ATID 34AC");
    delay(10);

    //default channel
    Serial.println("ATCH E"); //channel 14
    delay(10);

    //packetisation
    Serial.println("ATRO A");
    delay(10);

    //cmd mode timeout
    Serial.println("ATCT 64"); //100*100ms
    delay(10);

    //cmd mode guard time
    Serial.println("ATGT 1f4"); //500ms
    delay(10);


    //writes the settings to hard memory
    Serial.println("ATWR");
    delay(10);

    //soft reboot
    Serial.println("ATFR");
    delay(500);

    //flush
    while (Serial.available())  Serial.read();
}




#define CBUFF_SIZE 8        //MUST be a power of 2
#define MAX_READ_BYTES 100  //nb of bytes to read before we leave this function (to avoid blocking)

/* handle the reading of the data from the xbee on the serial port, checksum test
 * return value : nb of bytes written in pRet, 0 on error (bad checksum) or non-detection of start sequence
 * pointer return : message and type in a structure
 * requires : Serial initialisation
 */
int Xbee_receive(sMsg *pRet){
    static uint8_t i;
    static uint8_t smallBuf[CBUFF_SIZE]={0};
    char lu;
    int count=0;
    int j;
    //Serial.read()==-1 <=> no data available, count to limit the time spend in the loop in case of spam, checksum to get out of the loop if it is correct AND the sender address id OK (if sender=0 it means it has been reset to 0 after reading the message)
    while( Serial.available() \
            && count<=MAX_READ_BYTES \
            &&  ( cbChecksumHead(smallBuf,CBUFF_SIZE,(i-1)&(CBUFF_SIZE-1)) || !( smallBuf[(i-5)&(CBUFF_SIZE-1)]<<8&smallBuf[(i-4)&(CBUFF_SIZE-1)] ) ) ) {
        smallBuf[i]=Serial.read();
        i=(i+1)&(CBUFF_SIZE-1);                                          // &7 <~> %8, but better behaviour with negative in our case (and MUCH faster)
        count++;
    }


    if (count<=MAX_READ_BYTES && cbChecksumHead(smallBuf,CBUFF_SIZE,(i-1)&(CBUFF_SIZE-1)) && smallBuf[(i-3)&3] ){

        count=sizeof(sGenericHeader);
        //TODO : do we copy the message before checking payload checksum, or after (in that case, we must copy it one more time in memory if we don't want to corrupt any previous message stored at the location pointed by return pointer) ?

        //we copy the header in the return structure
        for (j=0;j<sizeof(sGenericHeader);j++){
            ((uint8_t *)(&(pRet->header)))[j]=smallBuf[(i-sizeof(sGenericHeader)+j)&(CBUFF_SIZE-1)];
        }

        //we read the rest of the data in this message (given by the "size" field of the header) and write the in the return structure
        count+=Serial.readBytes((char *)(&(pRet->payload)),pRet->header.size);
        memset(smallBuf,0,sizeof(smallBuf));
        return count;
    }

    return 0;
}


/*
 * Handle the sending of the message to the xbee via the serial.
 * Size & checksum of msg must be set before calling txXbee
 * Argument :
 *  msg : message to send (thank captain obvious!)
 * Return value :
 *  number of bytes writen (0 if error)
 */
int Xbee_send(sMsg msg){
    return  Serial.write((uint8_t *)&msg,msg.header.size+sizeof(sGenericHeader));
}
