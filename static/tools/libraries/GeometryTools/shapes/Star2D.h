/*
 * Star2D.h
 *
 *  Created on: 16 avr. 2015
 *      Author: ludo6431
 */

#ifndef LIB_GEOMETRYTOOLS_SHAPES_STAR2D_H_
#define LIB_GEOMETRYTOOLS_SHAPES_STAR2D_H_

#include <shapes/Polygon2D.h>

template<typename T>
class Star2D: public Polygon2D<T> {
public:
    Star2D();
    virtual ~Star2D();
};

#endif /* LIB_GEOMETRYTOOLS_SHAPES_STAR2D_H_ */
