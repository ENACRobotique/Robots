/*
 * PointOrient2D.h
 *
 *  Created on: 11 juin 2015
 *      Author: Sebastien Malissard
 */

#ifndef LIB_GEOMETRYTOOLS_POINTORIENT2D_H_
#define LIB_GEOMETRYTOOLS_POINTORIENT2D_H_

#include <cmath>
#include "Point2D.h"

template<typename T>
class Point2D;

template<typename T>
class PointOrient2D {
public:
    Point2D<T> p;
    T o;

    PointOrient2D() :
            p(0,0), o(0) {
    }
    PointOrient2D(const Point2D<T>& _p, const T _o) :
            p(_p), o(_o) {
    }
    PointOrient2D(const T _x, const T _y, const T _o) :
            p(_x, _y), o(_o) {
    }
    ~PointOrient2D(){
    }

	T getO() const {
		return o;
	}

private:
};




#endif /* LIB_GEOMETRYTOOLS_POINTORIENT2D_H_ */
