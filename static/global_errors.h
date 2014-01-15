/*
 * sb_errors.h
 *
 *  Created on: 16 déc. 2013
 *      Author: quentin
 */

#ifndef GLOBAL_ERRORS_H_
#define GLOBAL_ERRORS_H_

//!\ first error must be equal to 1.
// every function returning an error MUST return "-ERR_XXXXXX"

enum{
    // BOTNET specific errors
        ERR_BN_INIT_FAILED=1,
        // NETWORK ERRORS
        ERR_BN_NACK_BROKEN_LINK,
        ERR_BN_NACK_FULL_BUFFER,
        ERR_BN_ACK_TIMEOUT,
        ERR_BN_TYPE_TOO_HIGH,
        ERR_BN_TYPE_VERSION,
        // PING ERRORS
        ERR_BN_TIMEOUT,


    // UART (and UART_FRAMING) errors
        ERR_UART_OVERSIZE,          // packet too big to send
        ERR_UART_CSUM,              // frame rejected (incorrect checksum)
        ERR_UART_WRITE_BYTE,
        ERR_UART_WRITE_TIMEOUT,
        ERR_UART_READ_BYTE,
        ERR_UART_READ_BYTE_TIMEOUT,
        ERR_UART_INIT_FAILED,
        ERR_UART_DEINIT_FAILED,

    // XBEE_API errors
        ERR_XBEE_NOSTAT,    // status frame not received
        ERR_XBEE_NOACK,     // status frame received, but not acknoledged
        ERR_XBEE_CSUM,      // frame rejected (incorrect checksum)
        ERR_XBEE_INIT_FAILED,


    // Generaml errors
    ERR_INSUFFICIENT_MEMORY,

    ERR_UNKNOWN_ERROR,
    ERR_NUMBERRCODES
};

#endif /* GLOBAL_ERRORS_H_ */
