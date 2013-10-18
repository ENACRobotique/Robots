/*
 * lib_UART_x86.c
 *
 *  Created on: 28 mai 2013
 *      Author: quentin
 */

#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <fcntl.h>
#include <termios.h>
#include <stdlib.h>
#include <ctype.h>
#include <unistd.h>
#include <errno.h>
#include <sys/select.h>

#include "lib_UART_x86.h"
#include "lib_checksum.h"
#include "messages.h"

int UART_serial_port;

#define delay(d) usleep((d)*1000)
#define Serialprint(s) write(UART_serial_port, (s), strlen(s))
#define Serialprintln(s) do { write(UART_serial_port, (s), strlen(s)); write(UART_serial_port, "\n", 1); } while(0)

void setupUART(){
#if 0
    //flush
    delay(1000);
//    while (Serial.available()){ //flush
//        Serial.read();
//    }
    Serialprint("+++");
    delay(1000);
    //network cfg
    //each node is an end device, with end device association disabled (factory config)
    //Serial.print("ATCE0,A10");


    //networkID
    Serialprintln("ATID 34AC");
    delay(10);

    //default channel
    Serialprintln("ATCH E"); //channel 14
    delay(10);

    //packetisation
    Serialprintln("ATRO A");
    delay(10);

    //cmd mode timeout
    Serialprintln("ATCT 64"); //100*100ms
    delay(10);

    //cmd mode guard time
    Serialprintln("ATGT 1f4"); //500ms
    delay(10);


    //writes the settings to hard memory
    Serialprintln("ATWR");
    delay(10);

    //soft reboot
    Serialprintln("ATFR");
    delay(500);

    //flush
    char buf[32];
    while (read(UART_serial_port, buf, sizeof(buf)) > 0);
#endif
}

#undef delay
#undef Serialprint
#undef Serialprintln


/* Initialize the serial port designated by devStr
 * Arguments :
 *  devStr : string containing the location of the device/file
 *
 */
void UART_initSerial(char * devStr){
    struct termios options;

    if(!strcmp(devStr, "-")){ // reading from standard input
        UART_serial_port = 0;
        return;
    }

    UART_serial_port = open(devStr, O_RDWR | O_NOCTTY | O_NDELAY);//lecture et ecriture | pas controlling terminal | ne pas attendre DCD

    //cas d'erreur d'ouverture
    if(UART_serial_port < 0){
        perror("Erreur d'ouverture du port serie");
        exit(-1);
    }

    //chargement des données
    tcgetattr(UART_serial_port, &options);
    //B115200 bauds
    cfsetospeed(&options, B57600);
    options.c_cflag |= (CLOCAL | CREAD);//programme propriétaire du port
    //structure en 8N1 !!
    options.c_cflag &= ~PARENB; //pas de parité
    options.c_cflag &= ~CSTOPB; // 1 bit de stop
    options.c_cflag &= ~CSIZE; //option a 0
    options.c_cflag |= CS8; //8 bits
    tcsetattr(UART_serial_port, TCSANOW, &options); //enregistrement des valeurs de configuration

    fcntl(UART_serial_port,F_SETFL,10);//mode bloquant pour la fonction read() si aucun caractere dispo, programme attend
}


/* closes the serial port
 * Arguments : none
 *
 */
void UART_deInitSerial(){
    if(UART_serial_port){
        close(UART_serial_port);
    }
}


#define CBUFF_SIZE 8        //MUST be a power of 2
#define MAX_READ_BYTES 100  //nb of bytes to read before we leave this function (to avoid blocking)





/*
 *
 * /!\ Blocking function
 *
 */
int UART_receive(sMsg *pRet){
    static uint8_t i=0;
    static uint8_t smallBuf[CBUFF_SIZE]={0};
    int count=0;
    int j;
    int ret;


    fd_set s;
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 1*1000;

    FD_ZERO(&s);
    FD_SET(UART_serial_port, &s);
    if(select(UART_serial_port+1, &s, NULL, NULL, &tv) <= 0){
        return 0; // nothing to read or error
    }

    //read(...)=0 <=> no data available, <0 <=> error | count to limit the time spend in the loop in case of spam, checksum to get out of the loop if it is correct AND the sender address id OK (if sender=0 it means it has been reset to 0 after reading the message)
    while( (ret=read(UART_serial_port,&(smallBuf[i]),1))>0 \
            &&  ( !cbChecksumHead(smallBuf,CBUFF_SIZE,(i)&(CBUFF_SIZE-1)) || !( (smallBuf[(i-5)&(CBUFF_SIZE-1)]<<8) | smallBuf[(i-4)&(CBUFF_SIZE-1)] ) ) ) {
        i=(i+1)&(CBUFF_SIZE-1);                                          // &7 <~> %8, but better behaviour with negative in our case (and MUCH faster)
        count++;
        if(count >= MAX_READ_BYTES){
            break;
        }
    }

    if (ret==-1){
        perror("serial port reading");
        exit(-1);
    }

    if (count<=MAX_READ_BYTES && cbChecksumHead(smallBuf,CBUFF_SIZE,(i)&(CBUFF_SIZE-1)) &&  ( (smallBuf[(i-5)&(CBUFF_SIZE-1)]<<8) | smallBuf[(i-4)&(CBUFF_SIZE-1)] )  ){
        count=sizeof(sGenericHeader);

        //we copy the header in the return structure
        for (j=0;j<sizeof(sGenericHeader);j++){
            ((uint8_t *)(&(pRet->header)))[j]=smallBuf[(i-sizeof(sGenericHeader)+j+1)&(CBUFF_SIZE-1)];
        }

        //we read the rest of the data in this message (given by the "size" field of the header) and write the in the return structure
        while(count < pRet->header.size + sizeof(sGenericHeader)){
            count+=read(UART_serial_port,((char*)pRet) + count,pRet->header.size + sizeof(sGenericHeader) - count);
        }

        memset(smallBuf,0,sizeof(smallBuf));
        if (checksumPload(pRet)) return count;
        else return 0;
    }

    return 0;
}

int UART_send(sMsg *msg){
    int ret;
    int sent = 0;
    int len = sizeof(sGenericHeader) + msg->header.size;

    uint8_t *p = (uint8_t *)msg;

    do {
        ret = write(UART_serial_port?:2, p + sent, len - sent); // if reading from stdin, writing to stderr
        if(ret > 0) {
            sent += ret;
        }
    } while(ret > 0 && sent < len);

    if(ret < 0) {
        perror("serial port writing");
        exit(-1);
    }

    return sent;
}

