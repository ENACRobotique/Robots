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
    //TODO
    return -1;
}

int UART_send(sMsg *msg){
    //TODO

    return -1;
}

