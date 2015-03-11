/*
 * polygon2D.h
 *
 *  Created on: 8 mars 2015
 *      Author: Sebastien Malissard
 */

#ifndef LIB_GEOMETRYTOOLS_POLYGON2D_H_
#define LIB_GEOMETRYTOOLS_POLYGON2D_H_

#include <vector>

#include "Line2D.h"
#include "Point2D.h"

template<typename T>
class Polygon2D {
    public:
        Polygon2D();
        ~Polygon2D();

        void checkPtInPolygon(const Point2D<T> pt, bool& ret) const;

        std::vector <Point2D<T>> p;
};

#endif /* LIB_GEOMETRYTOOLS_POLYGON2D_H_ */
