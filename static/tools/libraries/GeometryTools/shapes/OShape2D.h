/*
 * OShape2D.h
 *
 *  Created on: 16 avr. 2015
 *      Author: ludo6431
 */

#ifndef LIB_GEOMETRYTOOLS_SHAPES_OSHAPE2D_H_
#define LIB_GEOMETRYTOOLS_SHAPES_OSHAPE2D_H_

template<typename T>
class OShape2D: public Shape2D<T> {
protected:
    Shape2D<T>* s;

public:
    OShape2D(Shape2D<T>* s);
    virtual ~OShape2D();
};

#endif /* LIB_GEOMETRYTOOLS_SHAPES_OSHAPE2D_H_ */
