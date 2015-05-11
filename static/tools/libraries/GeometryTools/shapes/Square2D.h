/*
 * Square2D.h
 *
 *  Created on: 16 avr. 2015
 *      Author: ludo6431
 */

#ifndef LIB_GEOMETRYTOOLS_SQUARE2D_H_
#define LIB_GEOMETRYTOOLS_SQUARE2D_H_

#include <shapes/Rect2D.h>

template<typename T>
class Square2D: public Rect2D<T> {
public:
    Square2D();
    virtual ~Square2D();
};

#endif /* LIB_GEOMETRYTOOLS_SQUARE2D_H_ */
