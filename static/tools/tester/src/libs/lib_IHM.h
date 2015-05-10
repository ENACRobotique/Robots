/*
 * state_blink.h
 *
 *  Created on: 15 mai 2013
 *      Author: quentin
 */

#ifndef STATE_BLINK_H_
#define STATE_BLINK_H_

#include <state_types.h>
#include "Encoder.h"

extern Encoder myEnc;

void afficher(const char * format...) __attribute__((format (printf, 1, 2)));

void ret();

#endif /* STATE_BLINK_H_ */
