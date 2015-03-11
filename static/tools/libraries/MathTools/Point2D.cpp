/*
 * Point2D2D.cpp
 *
 *  Created on: 8 mars 2015
 *      Author: Sébastien Malissard
 */

#include "Point2D.h"

#include <cmath>

template<typename T>
Point2D<T>::Point2D() : x(0), y(0) {
}

template<typename T>
Point2D<T>::Point2D(T _x, T _y) : x(_x), y(_y) {
}

template<typename T>
Point2D<T>::~Point2D() {
}

template<typename T>
T Point2D<T>::sqdistPt2Pt(const Point2D<T>& p) const{

    return (p.x - x) * (p.x - x) + (p.y - y) * (p.y - y);
}

template<typename T>
T Point2D<T>::distPt2Pt(const Point2D<T>& p) const{

    return sqrt((p.x - x) * (p.x - x) + (p.y - y) * (p.y - y));
}

template<typename T>
void Point2D<T>::translation(const T& _x, const T& _y){

    x += _x;
    y += _y;
}

template<typename T>
void Point2D<T>::rotation(const T& a){
    Point2D<T> pt = *this;

    x = pt.x*cos(a) - pt.y*sin(a);
    y = pt.x*sin(a) + pt.y*cos(a);
}

