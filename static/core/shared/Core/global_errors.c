#include "global_errors.h"

const char* getErrorStr(int err){
    if(err < 0){
        err = -err;
    }

    switch (err) {
    // General errors
    case ERR_MOAERR:                                    return "MOAERR";
    case ERR_INSUFFICIENT_MEMORY:                       return "INSUFFICIENT_MEMORY";
    case ERR_NOT_FOUND:                                 return "NOT_FOUND";
    case ERR_TRY_AGAIN:                                 return "TRY_AGAIN";              // eg. : try again later
    case ERR_UNKNOWN_ERROR:                             return "UNKNOWN_ERROR";
    case ERR_WRONG_FLAG:                                return "WRONG_FLAG";
    case ERR_ALREADY_IN_USE:                            return "ALREADY_IN_USE";
    case ERR_OUT_OF_BOND:                               return "OUT_OF_BOND";
    case ERR_SYSERRNO:                                  return "SYSERRNO";
    case ERR_NULL_POINTER_WRITE_ATTEMPT:                return "NULL_POINTER_WRITE_ATTEMPT";

    // BOTNET specific errors
    case ERR_BN_INIT_FAILED:                            return "BN_INIT_FAILED";
    case ERR_BN_NACK_BROKEN_LINK:                       return "BN_NACK_BROKEN_LINK";
    case ERR_BN_NACK_FULL_BUFFER:                       return "BN_NACK_FULL_BUFFER";
    case ERR_BN_ACK_TIMEOUT:                            return "BN_ACK_TIMEOUT";
    case ERR_BN_TYPE_TOO_HIGH:                          return "BN_TYPE_TOO_HIGH";
    case ERR_BN_TYPE_VERSION:                           return "BN_TYPE_VERSION";
    case ERR_BN_TYPE_ALREADY_ATTACHED:                  return "BN_TYPE_ALREADY_ATTACHED";
    case ERR_BN_WRONG_TYPE:                             return "BN_WRONG_TYPE";
    case ERR_BN_OVERSIZE:                               return "BN_OVERSIZE";
    case ERR_BN_CSUM:                                   return "BN_CSUM";
    case ERR_BN_NO_SUCH_INTERFACE:                      return "BN_NO_SUCH_INTERFACE";   // no such interface on this device
    case ERR_BN_UNKNOWN_ADDR:                           return "BN_UNKNOWN_ADDR";
    case ERR_BN_NO_LCAST_ADDR:                          return "ERR_BN_NO_LCAST_ADDR";   // the given address is not a linkcast one
    case ERR_BN_BUFFER_FULL:                            return "BN_BUFFER_FULL";

    // UART (and UART_FRAMING) errors
    case ERR_UART_OVERSIZE:                             return "UART_OVERSIZE";   // packet too big to send (size over UART_MTU)
    case ERR_UART_CSUM:                                 return "UART_CSUM";      // incoming frame rejected (incorrect checksum)
    case ERR_UART_WRITE_BYTE:                           return "UART_WRITE_BYTE";
    case ERR_UART_WRITE_TIMEOUT:                        return "UART_WRITE_TIMEOUT";
    case ERR_UART_READ_BYTE:                            return "UART_READ_BYTE";
    case ERR_UART_READ_BYTE_TIMEOUT:                    return "UART_READ_BYTE_TIMEOUT";

    // XBEE errors
    case ERR_XBEE_NOSTAT:                               return "XBEE_NOSTAT";    // status frame not received
    case ERR_XBEE_NOACK:                                return "XBEE_NOACK"; // status frame received and transmitted frame not acknoledged (may have been purged or CCAfail)
    case ERR_XBEE_CSUM:                                 return "XBEE_CSUM";      // frame rejected (incorrect checksum)
    case ERR_XBEE_OVERSIZE:                             return "XBEE_OVERSIZE";
    case ERR_XBEE_AT_WRONG_CMD:                         return "XBEE_AT_WRONG_CMD";  //AT cmd wrong command
    case ERR_XBEE_AT_WRONG_PAR:                         return "XBEE_AT_WRONG_PAR";  //AT cmd wrong param
    case ERR_XBEE_AT_ERR:                               return "XBEE_AT_ERR";        //AT cmd error
    case ERR_XBEE_INIT_FAILED:                          return "XBEE_INIT_FAILED";

    // I²C error
    case ERR_I2C_END_TX:                                return "I2C_END_TX";
    default:                                            return "UNKNOWN";
    }
}
