/*
 * Vector2D.cpp
 *
 *  Created on: 8 mars 2015
 *      Author: Sébastien Malissard
 */

#include "Vector2D.h"
#include <cmath>

Vector2D::Vector2D() : x(0), y(0) {
}

Vector2D::Vector2D(const Point2D& a, const Point2D& b) : x(b.x - a.x), y(b.y - a.y){
}

Vector2D::Vector2D(float _x, float _y) : x(_x), y(_y) {
}

Vector2D::~Vector2D() {
}
ERROR Vector2D::normVec(float& n)const{

    n = sqrt(x * x + y * y);

    return 0;
}

ERROR Vector2D::convPts2Vec(const Point2D& a, const Point2D& b){

    x = b.x - a.x;
    y = b.y - a.y;

    return 0;
}

ERROR Vector2D::dotVecs(const Vector2D& v, float& d) const{

    d = x * v.x + y * v.y;

    return 0;
}

ERROR Vector2D::crossVecs(const Vector2D& v, float& c) const{

    c = x * v.y - y * v.x;

    return 0;
}

ERROR Vector2D::rotVec(const float& theta){
    Vector2D vc = *this;

    x = vc.x * cos(theta) - vc.y * sin(theta);
    y = vc.x * sin(theta) + vc.y * cos(theta);

    return 0;
}
