/*
 * lib_line.h
 *
 *  Created on: 27 avr. 2013
 *      Author: quentin
 * 	 Modify on: janvier 2014
 * 	 		by: Seb
 */

#ifndef LIB_LINE_H_
#define LIB_LINE_H_

#define LINE_THRESHOLD 3500

uint16_t getIntensity(uint8_t ch);
void asserLine(void);
int lineDetection(void);

#endif /* LIB_LINE_H_ */
