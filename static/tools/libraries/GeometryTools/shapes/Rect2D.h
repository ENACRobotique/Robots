/*
 * Rect2D.h
 *
 *  Created on: 16 avr. 2015
 *      Author: ludo6431
 */

#ifndef LIB_GEOMETRYTOOLS_RECT2D_H_
#define LIB_GEOMETRYTOOLS_RECT2D_H_

#include <shapes/Polygon2D.h>

template<typename T>
class Rect2D: public Polygon2D<T> {
public:
    Rect2D();
    virtual ~Rect2D();
};

#endif /* LIB_GEOMETRYTOOLS_RECT2D_H_ */
