/*
 * Vector2D.h
 *
 *  Created on: 8 mars 2015
 *      Author: Sébastien Malissard
 */

#ifndef LIB_GEOMETRYTOOLS_VECTOR2D_H_
#define LIB_GEOMETRYTOOLS_VECTOR2D_H_

#include <cmath>
#include <iostream>

template<typename T>
class Point2D;

template<typename T>
class Vector2D {
public:
    T x, y;

    Vector2D() :
            x(0), y(0) {
    }
    Vector2D(const Vector2D& v) :
            x(v.x), y(v.y) {
    }
    Vector2D(const T _x, const T _y) :
            x(_x), y(_y) {
    }
    Vector2D(const Point2D<T>& a, const Point2D<T>& b) :
            x(b.x - a.x), y(b.y - a.y) {
    }
    ~Vector2D() {
    }

    T operator*(const Vector2D& v) const { //dot product
        return x * v.x + y * v.y;
    }
    T operator^(const Vector2D& v) const { // cross product
        return x * v.y - y * v.x;
    }

    Vector2D operator+(const Vector2D& v) const {
        return {x + v.x, y + v.y};
    }
    Vector2D operator-(const Vector2D& v) const {
        return {x - v.x, y - v.y};
    }
    Vector2D operator*(const T& r) const {
        return {x * r, y * r};
    }
    Vector2D operator/(const T& r) const {
        return {x / r, y / r};
    }

    Vector2D& operator+=(const Vector2D& v) {
        return *this = *this + v;
    }
    Vector2D& operator-=(const Vector2D& v) {
        return *this = *this - v;
    }
    Vector2D& operator*=(const T& r) {
        return *this = *this * r;
    }
    Vector2D& operator/=(const T& r) {
        return *this = *this / r;
    }
    Vector2D& operator=(const Vector2D& v) {
        x = v.x;
        y = v.y;
        return *this;
    }

    bool operator==(const Vector2D& v) const {
        return v.x == x && v.y == y;
    }
    bool operator!=(const Vector2D& v) const {
        return !(*this == v);
    }

    T norm() const {
        return std::hypot(x, y);
    }
    T normSq() const {
        return x * x + y * y;
    }

    void rotate(const T& theta) {
        Vector2D vc = *this;
        x = vc.x * std::cos(theta) - vc.y * std::sin(theta);
        y = vc.x * std::sin(theta) + vc.y * std::cos(theta);
    }
    T angleUnsigned(const Vector2D& v){
        return acos(*this * v / sqrt(normSq() * v.normSq()));
    }
    T angle(const Vector2D& v){ //'*this' is the reference
        return ((*this ^ v) > 0 ? 1 : -1) * angleUnsigned(v);
    }
};

template<typename T>
Vector2D<T> operator*(const T& n, const Vector2D<T>& v) {
    return v * n;
}

template<typename T>
std::ostream& operator<<(std::ostream& out, Vector2D<T>& v) {
    out << "(" << v.x << ";" << v.y << ")";
    return out;
}

#endif /* LIB_GEOMETRYTOOLS_VECTOR2D_H_ */
