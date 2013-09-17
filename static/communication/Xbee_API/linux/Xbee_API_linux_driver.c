/*
 * Xbee_API_linux_driver.c
 *
 *  Created on: 16 juin 2013
 *      Author: quentin
 */

#include <stdio.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <termios.h>
#include <unistd.h>
#include <stdint.h>
#include <stdlib.h>
#include <errno.h>

#include "Xbee_API_linux_drivers.h"
#include "params.h"

int Xbee_serial_port;

/* Expected behavior of serialInit :
 *  Initializes the serial communication
 */
int serialInit(uint32_t speed, void *device){
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
}

/* Expected behavior of serialDeInit :
 *  DeInitializes the serial communication
 */
int serialDeInit(){
    close(Xbee_serial_port);
    return 1;
}

/* Expected behavior of serialWrite
 *  Writes the content of byte to the serial link
 *  Returns 1 if success, 0 otherwise
 */
int serialWrite(uint8_t byte){
#ifdef DEBUG_PRINT_HEX
    printf("%x ",byte);
#endif
    int i;
    i=write(Xbee_serial_port, &byte, 1);
    fflush(NULL);
    return i;
}

/* Expected behavior of serialWrite
 *  Writes size byte to the serial link, beginning at data
 *  Returns number of bytes written
 */
int serialNWrite(const uint8_t *data,int size){
    int i;
    i=write(Xbee_serial_port, data, size);
    fflush(NULL);
    return i;
}

/* Expected behavior of serialRead
 *  Reads one byte from Serial, writes it at byte
 *  Returns 1 if success, 0 otherwise
 *  MUST be nonblocking
 */
int serialRead(uint8_t *byte){
    int i;
    i=read(Xbee_serial_port,byte,1);
    if (i<0) {
        if (errno==EAGAIN || errno==EWOULDBLOCK) return 0;
        else {
            perror("serialRead");
            exit(-1);
        }
    }
#ifdef DEBUG_PRINT_HEX
    else if (i) printf("%x ",*byte);
#endif
    return i;

}

/* Expected behavior of serialNRead
 *  Reads at most size byte from Serial, writes them at byte
 *  Returns number of bytes written
 *  MUST be nonblocking
 */
int serialNRead(uint8_t *data,int size){
    return read(Xbee_serial_port,data,size);
}

/* Expected behavior of testTimeout :
 *  1 - first call : start a timer of micros microsecond and return 1.
 *  2 - any call between the first call and "first call + micros"  : return 1.
 *  3 - first call after the end of the timer : return 0.
 *  next call : goto 1.
 *  testTimeout(0) MUST reset the timer : force next call to be in state 1
 *
 *  store is a pointer to a storing value, to enable nesting
 *
 *  /!\ watch out "store" for nesting /!\
 *  store must me initialized at 0
 *  and reinitialized at 0 before reuse
 */
int testTimeout(uint32_t delay, uint32_t *store){
    struct timeval currentClock;
    if (!delay) {
        *store=0;
        return 0;
    }
    if (!(*store)){
        gettimeofday(&currentClock,NULL);
        *store=currentClock.tv_sec*1000 + currentClock.tv_usec;
        return 1;
    }
    if (*store){
        gettimeofday(&currentClock,NULL);
        if ( (currentClock.tv_sec*1000 + currentClock.tv_usec) - *store >= delay){
            return 0;
        }
        else return 1;
    }
    return 0;
}

