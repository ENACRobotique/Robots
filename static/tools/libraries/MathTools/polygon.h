/*
 * polygon.h
 *
 *  Created on: 8 mars 2015
 *      Author: Sebastien Malissard
 */

#ifndef LIB_MATHTOOLS_POLYGON_H_
#define LIB_MATHTOOLS_POLYGON_H_

#include <vector>

#include "point.h"
#include "line.h"

class Polygon {
    public:
        Polygon();
        ~Polygon();

        ERROR checkPtInPoligon(const Point pt, bool& ret) const;

        std::vector <Point> p;
};

#endif /* LIB_MATHTOOLS_POLYGON_H_ */
