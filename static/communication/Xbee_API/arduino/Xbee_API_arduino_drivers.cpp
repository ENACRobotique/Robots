/*
 * Xbee_API_arduino_drivers.c
 *
 *  Created on: 17 juin 2013
 *      Author: quentin
 */


#include "Arduino.h"

#include "Xbee_API_arduino_drivers.h"


/* Expected behavior of serialInit :
 *  Initializes the serial communication
 */
int serialInit(uint32_t speed, void *device){

    Serial.begin(speed);
    Serial.setTimeout(0); // TODO check behavior immediate read, (faster) byte tempo done in Xbee_API.

    return 1;
}

/* Expected behavior of serialDeInit :
 *  DeInitializes the serial communication
 */
int serialDeInit(){
    return 0;
}

/* Expected behavior of serialWrite
 *  Writes the content of byte to the serial link
 *  Returns 1 if success, 0 otherwise
 */
int serialWrite(uint8_t byte){
    return Serial.write(byte);
}

/* Expected behavior of serialWrite
 *  Writes size byte to the serial link, beginning at data
 *  Returns number of bytes written
 */
int serialNWrite(const uint8_t *data,int size){
    return Serial.write(data, size);
}

/* Expected behavior of serialRead
 *  Reads one byte from Serial, writes it at byte
 *  Returns 1 if success, 0 otherwise
 *  MUST be nonblocking
 */
int serialRead(uint8_t *byte){
    if (Serial.available()) {
        *byte=Serial.read();
        return 1;
    }
    return 0;
}

/* Expected behavior of serialNRead
 *  Reads at most size byte from Serial, writes them at byte
 *  Returns number of bytes written
 *  MUST be nonblocking
 */
int serialNRead(uint8_t *data,int size){
    return Serial.readBytes((char *)data,size);
}

/* Expected behavior of testTimeout :
 *  1 - first call : start a timer of micros microsecond and return 1.
 *  2 - any call between the first call and "first call + micros"  : return 1.
 *  3 - first call after the end of the timer : return 0.
 *  next call : goto 1.
 *  testTimeout(0) MUST reset the timer : force next call to be in state 1
 *
 *
 *  store is a pointer to a storing value, to enable nesting
 *
 *  /!\ watch out "store" for nesting /!\
 */
int testTimeout(uint32_t delay, uint32_t *store){
    if (delay==0) {
        *store=0;
        return 0;
    }
    if (!*store){
        *store=micros();
        return 1;
    }
    if (*store){
        if ( (micros()-*store) >= delay ){
            return 1;
        }
    }
    return 0;

}


