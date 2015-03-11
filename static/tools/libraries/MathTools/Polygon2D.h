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

class Polygon2D {
    public:
        Polygon2D();
        ~Polygon2D();

        ERROR checkPtInPolygon(const Point2D pt, bool& ret) const;

        std::vector <Point2D> p;
};

#endif /* LIB_GEOMETRYTOOLS_POLYGON2D_H_ */
