/*
 * lib_UART_framing_linux.c
 *
 *  Created on: 2 nov. 2013
 *      Author: quentin
 */

#include "lib_UART_framing_linux.h"

#include <stdio.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <stdint.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/select.h>


int Xbee_serial_port;

/* initSerial : Initialises the UART serial interface (opens the serial interface at 115200 bauds nonblocking, in a way that pleases the Xbee)
 * Argument :
 *  device : string describing the device
 * Return value :
 *  >0 on success
 *  <0 on error
 */
int serialInit(const char *device){
    struct termios options;
        char *devStr=device;

        printf("opening of:%s: at 115200 bd\n",devStr);
        Xbee_serial_port = open(devStr, O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK);//lecture et ecriture | pas controlling terminal | ne pas attendre DCD

        //cas d'erreur d'ouverture
        if(Xbee_serial_port < 0)
        {
            perror("Erreur d'ouverture du port serie");
            exit(-1);
        }

        //chargement des données
        tcgetattr(Xbee_serial_port, &options);
        //B115200 bauds
        cfsetospeed(&options, B115200);
        options.c_cflag |= (CLOCAL | CREAD);//programme propriétaire du port
        options.c_cflag &= ~PARENB; //pas de parité
        options.c_cflag |= CSTOPB; // 2 bit de stop
        options.c_cflag &= ~CSIZE; //option a 0
        options.c_cflag |= CS8; //8 bits

        options.c_iflag |= IXON;    //enable flow control
        options.c_iflag |= IXOFF;   //enable flow control
        tcsetattr(Xbee_serial_port, TCSANOW, &options); //enregistrement des valeurs de configuration
        printf("Port serie ouvert\n");

        fcntl(Xbee_serial_port,F_SETFL,O_NONBLOCK);//mode non bloquant pour la fonction read() si aucun caractere dispo, programme attend

        return 1;
    return 1;
}

/* deinitSerial : Closes the UART serial interface
 * Argument :
 *  none
 * Return value :
 *  >0 on success
 *  <0 on error
 */
int serialDeinit(){
    close(Xbee_serial_port);
    return 1;
}

/* serialRead : reads one byte from serial interface
 * Argument :
 *  byte : pointer to where the byte should be stored
 *  timeout : in µs, time after which the function returns 0 if no byte can be read.
 * Return value :
 *  1 if one byte have been read
 *  0 if timeout
 *  -1 on error
 *
 */
int serialRead(uint8_t *byte,uint32_t timeout){
    int i;

    // some variables and struct to use select
    fd_set rfds;
    struct timeval tv;
    int retval;

    // Watch Xbee to see when it has input
    FD_ZERO(&rfds);
    FD_SET(Xbee_serial_port, &rfds);

    // Wait up to timeout microseconds.
    tv.tv_sec = timeout/1000000;
    tv.tv_usec = timeout%1000000;

    // select() wait
    retval = select(Xbee_serial_port+1, &rfds, NULL, NULL, &tv);

    if (retval == -1){
       perror("select()");
    }
    else if (retval>0){
       // Data is available now, we can read
        i=read(Xbee_serial_port,byte,1);
        if (i<0) {
            if (errno==EAGAIN || errno==EWOULDBLOCK) return 0;
            else {
                perror("serialRead");
                exit(-1);
            }
        }
#ifdef DEBUG_PRINT_HEX
        else if (i) {
            printf("r%x ",*byte);
            fflush(stdout);
        }
#endif
        return i;

   }
   else return 0; //timeout

}

/* serialWrite : writes a byte on the serial interface
 * Argument :
 *  byte : byte to write
 * Return value :
 *  1 on success
 *  -1 on error
 */
int serialWrite(uint8_t byte){
#ifdef DEBUG_PRINT_HEX
    printf("w%x ",byte);
#endif
    int i;
    i=write(Xbee_serial_port, &byte, 1);
    fflush(NULL); //flushes all stream
    return (i==0)?-1:i;
}
