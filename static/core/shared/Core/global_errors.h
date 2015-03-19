/*
 * sb_errors.h
 *
 *  Created on: 16 déc. 2013
 *      Author: quentin
 */

#ifndef GLOBAL_ERRORS_H_
#define GLOBAL_ERRORS_H_

/*
 * Philosophy of this error definition :
 * Functions SHOULD return a value, >=0 of the expected result has been producted, <0 if error.
 * Error codes SHOULD be kept and returned by calling functions (ex : main() calls fun1() which calls fun2(). fun2() returns -ERR_YYY. fun1() should return -ERR_YYY.
 *
 */


//!\ first error must be equal to 1.
// every function returning an error MUST return "-ERR_XXXXXX"

enum{
    // General errors
    ERR_MOAERR=1,
    ERR_INSUFFICIENT_MEMORY,
    ERR_NOT_FOUND,
    ERR_TRY_AGAIN,              // eg. : try again later
    ERR_UNKNOWN_ERROR,
    ERR_WRONG_FLAG,
    ERR_ALREADY_IN_USE,
    ERR_OUT_OF_BOND,
    ERR_SYSERRNO,
    ERR_NULL_POINTER_WRITE_ATTEMPT, //10

    // BOTNET specific errors
    ERR_BN_INIT_FAILED,
    ERR_BN_NACK_BROKEN_LINK,
    ERR_BN_NACK_FULL_BUFFER,
    ERR_BN_ACK_TIMEOUT,
    ERR_BN_TYPE_TOO_HIGH,
    ERR_BN_TYPE_VERSION,
    ERR_BN_TYPE_ALREADY_ATTACHED,
    ERR_BN_WRONG_TYPE,
    ERR_BN_OVERSIZE,
    ERR_BN_CSUM,
    ERR_BN_NO_SUCH_INTERFACE,   //no such interface on this device
    ERR_BN_UNKNOWN_ADDR,
    ERR_BN_BUFFER_FULL, //23

    // UART (and UART_FRAMING) errors
    ERR_UART_OVERSIZE,          // packet too big to send (size over UART_MTU)
    ERR_UART_CSUM,              // incoming frame rejected (incorrect checksum)
    ERR_UART_WRITE_BYTE,
    ERR_UART_WRITE_TIMEOUT,
    ERR_UART_READ_BYTE,
    ERR_UART_READ_BYTE_TIMEOUT,     // 29

    // XBEE errors
    ERR_XBEE_NOSTAT,    // status frame not received
    ERR_XBEE_NOACK,     // status frame received and transmitted frame not acknoledged (may have been purged or CCAfail)
    ERR_XBEE_CSUM,      // frame rejected (incorrect checksum)
    ERR_XBEE_OVERSIZE,
    ERR_XBEE_AT_WRONG_CMD,  //AT cmd wrong command
    ERR_XBEE_AT_WRONG_PAR,  //AT cmd wrong param
    ERR_XBEE_AT_ERR,        //AT cmd error
    ERR_XBEE_INIT_FAILED, // 37

    // I²C error
    ERR_I2C_END_TX, // 38

    ERR_NUMBERRCODES
};

#endif /* GLOBAL_ERRORS_H_ */
