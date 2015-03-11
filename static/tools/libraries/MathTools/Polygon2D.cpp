/*
 * polygon.cpp
 *
 *  Created on: 8 mars 2015
 *      Author: Sebastien Malissard
 */

#include "Polygon2D.h"
#include <array>

template<typename T>
Polygon2D<T>::Polygon2D() {
}

template<typename T>
Polygon2D<T>::~Polygon2D() {
}

template<typename T>
ERROR Polygon2D<T>::checkPtInPolygon(const Point2D<T> pt, bool& ret) const{
  /*  std::array <Line, this->p.size()> l; //FIXME

    for (unsigned int i = 0; i < this->p.size(); i++) {
        bool norm = false;
        l[i].convPts2Line(this->p[i], this->p[(i + 1) % this->p.size()], norm);
    }

    int sg_prev = SIGN(l[0].a * pt.x + l[0].b * pt.y + l[0].c), sg = 0;

    for (unsigned int i = 1 ; i < this->p.size(); i++) {
        sg = SIGN(l[i].a * pt.x + l[i].b * pt.y + l[i].c);
        if (sg != sg_prev) {
            ret = false;
            return 0;
        }
    }

    ret = true;
*/
    return 0;
}
