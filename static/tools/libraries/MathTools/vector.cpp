/*
 * vector.cpp
 *
 *  Created on: 8 mars 2015
 *      Author: Sébastien Malissard
 */

#include "vector.h"

#include <cmath>

Vector::Vector() : x(0), y(0) {
}

Vector::Vector(const Point& a, const Point& b) : x(b.x - a.x), y(b.y - a.y){
}

Vector::Vector(float _x, float _y) : x(_x), y(_y) {
}

Vector::~Vector() {
}
ERROR Vector::normVec(float& n)const{

    n = sqrt(x * x + y * y);

    return 0;
}

ERROR Vector::convPts2Vec(const Point& a, const Point& b){

    x = b.x - a.x;
    y = b.y - a.y;

    return 0;
}

ERROR Vector::dotVecs(const Vector& v, float& d) const{

    d = x * v.x + y * v.y;

    return 0;
}

ERROR Vector::crossVecs(const Vector& v, float& c) const{

    c = x * v.y - y * v.x;

    return 0;
}

ERROR Vector::rotVec(const float& theta){
    Vector vc = *this;

    x = vc.x * cos(theta) - vc.y * sin(theta);
    y = vc.x * sin(theta) + vc.y * cos(theta);

    return 0;
}
