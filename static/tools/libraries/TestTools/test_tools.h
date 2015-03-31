/*
 * test_tools.h
 *
 *  Created on: 9 oct. 2014
 *      Author: ludo6431
 */

#include <math.h>

#ifndef LIB_TESTTOOLS_TEST_TOOLS_H_
#define LIB_TESTTOOLS_TEST_TOOLS_H_

#define IS_NEAR(a, b, eps) (fabs((a) - (b)) < (eps))
#define IS_NEAR_PERCENT(a, b, eps) (fabs(((a) - (b))/(a)) < (eps))

#define ASSERT_NEAR(a, b, eps) assert(IS_NEAR((a), (b), (eps)))
#define ASSERT_NEAR_PERCENT(a, b, eps) assert(IS_NEAR_PERCENT((a), (b), (eps)))

float rand_uniform(float min, float max);

#endif /* LIB_TESTTOOLS_TEST_TOOLS_H_ */
