/*
 * messages-image-processing.h
 *
 *  Created on: 15 April 2015
 *      Author: Yoann Solana
 */

#ifndef LIB_NETWORK_CONFIG_MESSAGES_IMAGE_PROCESSING_H_
#define LIB_NETWORK_CONFIG_MESSAGES_IMAGE_PROCESSING_H_

#include <stdint.h>


typedef struct __attribute__((packed)){
	float x;
	float f;
	float theta;
	int goCapture;  // To start a capture
} sPosCam;


#endif /* LIB_NETWORK_CONFIG_MESSAGES_IMAGE_PROCESSING_H_ */
