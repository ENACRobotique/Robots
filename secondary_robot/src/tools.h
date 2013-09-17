/*
 * tools.h
 *
 *  Created on: 27 avr. 2013
 *      Author: quentin
 */

#ifndef TOOLS_H_
#define TOOLS_H_


#ifndef CLAMP
#define CLAMP(m, n, M) min(max((m), (n)), (M))
#endif
#ifndef BIT
#define BIT(i) (1<<(i))
#endif

#endif /* TOOLS_H_ */
