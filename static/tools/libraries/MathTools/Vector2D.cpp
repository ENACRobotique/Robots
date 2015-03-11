/*
 * Vector2D.cpp
 *
 *  Created on: 8 mars 2015
 *      Author: Sébastien Malissard
 */

#include "Vector2D.h"
#include <cmath>

template<typename T>
Vector2D<T>::Vector2D() : x(0), y(0) {
}

template<typename T>
Vector2D<T>::Vector2D(const Point2D<T>& a, const Point2D<T>& b) : x(b.x - a.x), y(b.y - a.y){
}

template<typename T>
Vector2D<T>::Vector2D(T _x, T _y) : x(_x), y(_y) {
}

template<typename T>
Vector2D<T>::~Vector2D() {
}

template<typename T>
void Vector2D<T>::normVec(T& n)const{

    n = sqrt(x * x + y * y);
}

template<typename T>
void Vector2D<T>::convPts2Vec(const Point2D<T>& a, const Point2D<T>& b){

    x = b.x - a.x;
    y = b.y - a.y;
}

template<typename T>
void Vector2D<T>::dotVecs(const Vector2D& v, T& d) const{

    d = x * v.x + y * v.y;
}

template<typename T>
void Vector2D<T>::crossVecs(const Vector2D& v, T& c) const{

    c = x * v.y - y * v.x;
}

template<typename T>
void Vector2D<T>::rotVec(const T& theta){
    Vector2D vc = *this;

    x = vc.x * cos(theta) - vc.y * sin(theta);
    y = vc.x * sin(theta) + vc.y * cos(theta);
}
