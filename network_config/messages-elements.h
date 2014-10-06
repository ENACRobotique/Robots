/*
 * messages-elements.h
 *
 *  Created on: 6 oct. 2014
 *      Author: ludo6431
 */

#ifndef LIB_NETWORK_CONFIG_MESSAGES_ELEMENTS_H_
#define LIB_NETWORK_CONFIG_MESSAGES_ELEMENTS_H_

typedef enum{
    ELT_PRIMARY,
    ELT_SECONDARY,
    ELT_ADV_PRIMARY,
    ELT_ADV_SECONDARY,
    ELT_FIRE,
    ELT_ZONE,

    NUM_E_ELEMENT
} eElement;

int elementHasPosition(eElement id);

#endif /* LIB_NETWORK_CONFIG_MESSAGES_ELEMENTS_H_ */
