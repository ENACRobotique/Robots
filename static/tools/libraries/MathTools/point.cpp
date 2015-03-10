/*
 * Point.cpp
 *
 *  Created on: 8 mars 2015
 *      Author: Sébastien Malissard
 */

#include "point.h"

#include <cmath>

Point::Point() : x(0), y(0) {
}

Point::Point(float _x, float _y) : x(_x), y(_y) {
}

Point::~Point() {
}

ERROR Point::sqdistPt2Pt(const Point& p, float& d) const{

    d = (p.x - x) * (p.x - x) + (p.y - y) * (p.y - y);

    return 0;
}

ERROR Point::distPt2Pt(const Point& p, float& d) const{

    d = sqrt((p.x - x) * (p.x - x) + (p.y - y) * (p.y - y));

    return 0;
}

ERROR Point::equal(const Point& p, const float& tolerance, bool ret) const{
    float d;

    distPt2Pt(p, d);
    if(d <= tolerance)
        ret = true;
    else
        ret = false;

    return 0;
}

ERROR Point::translation(const float& _x, const float& _y){

    x += _x;
    y += _y;

    return 0;
}

ERROR Point::rotation(const float& a){
    Point pt = *this;

    x = pt.x*cos(a) - pt.y*sin(a);
    y = pt.x*sin(a) + pt.y*cos(a);

    return 0;
}

