/*
 * Ellipse2D.h
 *
 *  Created on: 16 avr. 2015
 *      Author: ludo6431
 */

#ifndef LIB_GEOMETRYTOOLS_SHAPES_ELLIPSE2D_H_
#define LIB_GEOMETRYTOOLS_SHAPES_ELLIPSE2D_H_

#include <shapes/Shape2D.h>

template<typename T>
class Ellipse2D: public Shape2D<T> {
public:
    Ellipse2D();
    virtual ~Ellipse2D();
};

#endif /* LIB_GEOMETRYTOOLS_SHAPES_ELLIPSE2D_H_ */
