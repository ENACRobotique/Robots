/*
 * lib_I2C_arduino.h
 *
 *  Created on: 6 juin 2013
 *      Author: quentin
 */

#ifndef LIB_I2C_ARDUINO_H_
#define LIB_I2C_ARDUINO_H_

#ifdef __cplusplus
extern "C" { //to enable use in both C projects an C++ projects
#endif


/*
 *
 * /!\ requires a Wire.begin(my_address_I2C)
 *
 */

int I2C_receive(sMsg *pRet);
int I2C_send(sMsg *msg,sb_Address firstDest);

#ifdef __cplusplus
}
#endif

#endif /* LIB_I2C_ARDUINO_H_ */
