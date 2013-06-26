/*
 * Xbee_API_linux_drivers.h
 *
 *  Created on: 16 juin 2013
 *      Author: quentin
 */

#ifndef XBEE_API_LINUX_DRIVERS_H_
#define XBEE_API_LINUX_DRIVERS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Expected behavior of serialInit :
 *  Initializes the serial communication
 */
int serialInit(uint32_t speed, void *device);

/* Expected behavior of serialDeInit :
 *  DeInitializes the serial communication
 */
int serialDeInit();

/* Expected behavior of serialWrite
 *  Writes the content of byte to the serial link
 *  Returns 1 if success, 0 otherwise
 */
int serialWrite(uint8_t byte);

/* Expected behavior of serialWrite
 *  Writes size byte to the serial link, beginning at data
 *  Returns number of bytes written
 */
int serialNWrite(const uint8_t *data,int size);

/* Expected behavior of serialRead
 *  Reads one byte from Serial, writes it at byte
 *  Returns 1 if success, 0 otherwise
 *  MUST be nonblocking
 */
int serialRead(uint8_t *byte);

/* Expected behavior of serialNRead
 *  Reads at most size byte from Serial, writes them at byte
 *  Returns number of bytes written
 *  MUST be nonblocking
 */
int serialNRead(uint8_t *data,int size);

/* Expected behavior of testTimeout :
 *  1 - first call : start a timer of micros microsecond and return 1.
 *  2 - any call between the first call and the end of the timer : return 1.
 *  3 - first call after the end of the timer : return 0.
 *  next call : goto 1.
 *  testTimeout(0) MUST reset the timer : force next call to be in state 1
 *
 *  /!\ DO NOT NEST /!\
 */
int testTimeout(uint32_t delay);

#ifdef __cplusplus
}
#endif

#endif /* XBEE_API_LINUX_DRIVERS_H_ */
