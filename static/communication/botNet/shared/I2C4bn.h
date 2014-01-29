/*
 * I2C4bn.h
 *
 *  Created on: 21 janv. 2014
 *      Author: quentin
 */

#ifndef I2C4BN_H_
#define I2C4BN_H_

#include "node_cfg.h"

#ifdef ARCH_328P_ARDUINO
    #include "../arduino/I2C/lib_I2C_arduino.h"
#elif defined(ARCH_X86_LINUX)
#error "I2C unavailable for linux for now"
#elif defined(ARCH_LPC21XX)
    #include "../lpc21xx/I2C/lib_I2C_lpc21xx.h"
#else
#error "please Define The Architecture Symbol, You Bloody Bastard"
#endif

#endif /* I2C4BN_H_ */
