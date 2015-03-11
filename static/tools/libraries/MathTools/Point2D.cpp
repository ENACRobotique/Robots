/*
 * Point2D2D.cpp
 *
 *  Created on: 8 mars 2015
 *      Author: Sébastien Malissard
 */

#include "Point2D.h"

#include <cmath>

Point2D::Point2D() : x(0), y(0) {
}

Point2D::Point2D(float _x, float _y) : x(_x), y(_y) {
}

Point2D::~Point2D() {
}

ERROR Point2D::sqdistPt2Pt(const Point2D& p, float& d) const{

    d = (p.x - x) * (p.x - x) + (p.y - y) * (p.y - y);

    return 0;
}

ERROR Point2D::distPt2Pt(const Point2D& p, float& d) const{

    d = sqrt((p.x - x) * (p.x - x) + (p.y - y) * (p.y - y));

    return 0;
}

ERROR Point2D::equal(const Point2D& p, const float& tolerance, bool ret) const{
    float d;

    distPt2Pt(p, d);
    if(d <= tolerance)
        ret = true;
    else
        ret = false;

    return 0;
}

ERROR Point2D::translation(const float& _x, const float& _y){

    x += _x;
    y += _y;

    return 0;
}

ERROR Point2D::rotation(const float& a){
    Point2D pt = *this;

    x = pt.x*cos(a) - pt.y*sin(a);
    y = pt.x*sin(a) + pt.y*cos(a);

    return 0;
}

