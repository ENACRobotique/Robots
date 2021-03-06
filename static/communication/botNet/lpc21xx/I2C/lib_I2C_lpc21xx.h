
#ifdef ARCH_LPC21XX

#ifndef _LIB_I2C_LPC21XX_H
#define _LIB_I2C_LPC21XX_H

#include "messages.h"

void I2C_init(unsigned long speed);
int I2C_receive(sMsg *pRet);
int I2C_send(const sMsg *msg, bn_Address firstDest);

#endif

#endif // ARCH_LPC21XX
