/*
 * Xbee_API.h
 *
 * This library shall enable the
 *
 *  Created on: 14 juin 2013
 *      Author: quentin
 */


/*
 * /!\ important remark :
 *  unless otherwise specified, variable endianess is unspecified
 *  "_be" suffix means "big-endian" for any value which should be understood by the Xbee (e.g. parameters of AT cmd parameters).
 *  "_be" does not apply on higher-level data.
 */
#ifndef XBEE_API_H_
#define XBEE_API_H_

#include <stdint.h>

#ifndef BIT
#define BIT(a) (1<<(a))
#endif


/////////////// Xbee specific variables

typedef uint16_t XbeeAddr16_t;  // 16-bit addresses
typedef uint64_t XbeeAddr64_t;  // 64-bit addresses

typedef struct{
    uint8_t frameID;            // for acknowledgment purposes
    uint8_t status;
}sTXStatus;

typedef struct{
    uint16_t lSrcAddr;          // LINK (layer 2) source address
    uint8_t rssi;
    uint8_t options;
    uint8_t payload[100];
}sRX16Data;

typedef struct{
    uint8_t     frameID;            // for acknowledgment purposes
    uint16_t    cmd_be;             // ascii big endian command. ex : for "DL", we MUST have cmb_be = ('D'<<8) | 'L'
    uint8_t     parameter_be[16];   // hexadecimal parameter value (if any) of the AT cmd. Value must be written big-endian.
}sATCmd;

typedef struct{
    uint8_t     frameID;            // for acknowledgment purposes
    uint16_t    cmd_be;             // ascii Command. ex : for "DL", we MUST have cmb_be = ( 'D'<<8 ) | 'L'
    uint8_t     status;
    uint8_t     value_be[16];        // hexa value returned (if any) by the AT cmd. Value must be written big-endian.
}sATResponse;

typedef union{
    sRX16Data RXdata;
    uint8_t modemStatus;
    sATResponse ATResponse;
    sATCmd ATCmd;
}uAPISpecificData;

typedef struct {
    uint8_t APID;           // API identifier
    uAPISpecificData data;
}sAPISpecificStruct;

typedef struct{
    uint8_t startB;             // 0x7E
    uint16_t length_be;         // length (big endian) of "data" field
    sAPISpecificStruct frameData;
    uint8_t checksum;
}sGenericXbeeFrame;

/////////////// Xbee specific defines and flags
    //// Special
    #define XBEE_FRAME_START    0x7E
    #define XBEE_ESCAPE_CHAR    0x7D
    #define XBEE_ESCAPE_MASK    0x20
    #define XBEE_XON            0x11
    #define XBEE_XOFF           0x13

    //// TX option flags
    #define XBEE_TX_O_NOACK     0x01
    #define XBEE_TX_O_BCAST     0x04    // use alone (withous other flags)
    //// TX status
    #define XBEE_TX_S_SUCCESS   0
    #define XBEE_TX_S_NOACK     1       // no ack received after all retries (ALWAYS the case if broadcast)
    #define XBEE_TX_S_CCAFAIL   2       // Clear Channel assessment fail
    #define XBEE_TX_S_PURGED    3       // Coordinator times out of an indirect transmission

    //// RX option flags
    #define XBEE_RX_O_ADDRBCAST 0x02
    #define XBEE_RX_O_PANBCAST  0x04

    //// Modem status
    #define XBEE_MODEM_S_HARDRST        0   // Hardware reset
    #define XBEE_MODEM_S_WATCHDOGRST    1   // Wtachdog timer reset
    #define XBEE_MODEM_S_ASSOC          2   // Associated
    #define XBEE_MODEM_S_DISASSOC       3   // Disassociated
    #define XBEE_MODEM_S_SYNCLOST       4   // Synchronization Lost (Beacon -enabled only)
    #define XBEE_MODEM_S_COORDREAL      5   // Coordinator realignment
    #define XBEE_MODEM_S_COORDSTART     6   // Coordinator started

    //// AT response status
    #define XBEE_ATR_S_OK
    #define XBEE_ATR_S_ERROR
    #define XBEE_ATR_S_INVCOM   // Invalid command
    #define XBEE_ATR_S_INVPAR   // Invalid parameter


    //// API Identifiers
    #define XBEE_APID_MODEM_S           0x8A    // Modem status
    #define XBEE_APID_ATCMD             0x08    // AT command immediately applied
    #define XBEE_APID_ATCMD_QUEUE       0x09    // AT command applied after "AC" command only
    #define XBEE_APID_ATRESPONSE        0x88    // response to an AT cmd
    #define XBEE_APID_REMOTE_ATCMD      0x17    // remote AT command XXX not implemented
    #define XBEE_APID_REMOTE_ETRESPONSE 0x97    // response to remote AT command XXX not implemented
    #define XBEE_APID_TX64              0x00    // 64 bits address TX request XXX not implemented
    #define XBEE_APID_TX16              0x01    // 16 bits address TX request
    #define XBEE_APID_TXS               0x89    // TX status 16
    #define XBEE_APID_RX64              0x80    // 64 bits address RX XXX not implemented
    #define XBEE_APID_RX16              0x81    // 16 bits address RX
    #define XBEE_APID_RXIO64            0x82    // 64 bits address IO RX XXX not implemented
    #define XBEE_APID_RXIO16            0x83    // 16 bits address IO RX XXX not implemented

/* Xbee TX request for 16bits addresses
 * Return value : nb of bytes written to serial port, 0 if error
 */
int XbeeTx16(XbeeAddr16_t to,uint8_t options, uint8_t frameID, void* data, uint16_t datasize);

/* Xbee TX status reading
 * Return value : nb of bytes red from serial port, 0 if error
 * status written in *status
 */
int XbeeTxStatus(sTXStatus *status);

int XbeeATCmd(char cmd[2],int val_be);

int XbeeWriteFrame(uint8_t apid, uint16_t size, uint8_t *data_be);

int XbeeReadFrame(sAPISpecificStruct *str);
int XbeeWriteByteEscaped(uint8_t byte);
int XbeeReadByteEscaped(uint8_t *byte);


#endif /* XBEE_API_H_ */
