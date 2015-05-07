/*
 * Shape2D.h
 *
 *  Created on: 16 avr. 2015
 *      Author: ludo6431
 */

#ifndef LIB_GEOMETRYTOOLS_SHAPE2D_H_
#define LIB_GEOMETRYTOOLS_SHAPE2D_H_

#include <Point2D.h>
#include <shapes/Rect2D.h>

template<typename T>
class Shape2D {
public:
    Shape2D();
    virtual ~Shape2D();

    virtual Point2D<T> getCenter() const = 0;
    virtual Rect2D<T> getBoundingBox() const = 0;
    virtual bool contains(const Point2D<T>& pt) const = 0;
};

#endif /* LIB_GEOMETRYTOOLS_SHAPE2D_H_ */
