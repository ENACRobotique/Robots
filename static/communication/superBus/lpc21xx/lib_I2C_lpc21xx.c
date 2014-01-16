#include <stdio.h>

#include "network_cfg.h" // DEVICEI_MASK
#include "node_cfg.h" // MYADDRI

// from lpc2148/lpc_lib
#include "i2c.h" // i2c_* lpc functions

#include "lib_I2C_lpc21xx.h"

void I2C_init(unsigned long speed) {
    i2c_init(speed, MYADDRI & DEVICEI_MASK);
}

int I2C_receive(sMsg *pRet) {
    return i2c_recvchunk(NULL, (uint8_t *)pRet, sizeof(*pRet));
}

int I2C_send(const sMsg *msg, sb_Address firstDest){
    return i2c_sendchunk(firstDest & DEVICEI_MASK, (const uint8_t *)msg, sizeof(msg->header) + msg->header.size);
}

