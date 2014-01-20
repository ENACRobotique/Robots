/*
 * timeout.h
 *
 *  Created on: 1 nov. 2013
 *      Author: quentin
 */

#ifndef TIMEOUT_H_
#define TIMEOUT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

int testTimeout(uint32_t delay, uint32_t *store);
uint32_t stopwatch(uint32_t *store);

#ifdef __cplusplus
}
#endif

#endif /* TIMEOUT_H_ */
