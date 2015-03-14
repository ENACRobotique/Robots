/*
 * Vector2D.h
 *
 *  Created on: 8 mars 2015
 *      Author: Sébastien Malissard
 */

#ifndef LIB_GEOMETRYTOOLS_VECTOR2D_H_
#define LIB_GEOMETRYTOOLS_VECTOR2D_H_

#include <iostream>
#include <cmath>

template<typename T>
class Point2D;

template<typename T>
class Vector2D {
    public:
        Vector2D() : x(0), y(0){}
        Vector2D(const Vector2D<T>& v) : x(v.x), y(v.y){}
        Vector2D(const T _x, const T _y) : x(_x), y(_y){}
        Vector2D(const Point2D<T>& a, const Point2D<T>& b) : x(b.x - a.x), y(b.y - a.y){}
        ~Vector2D(){}

        T operator*(const Vector2D& v) const{ //dot product
            return x * v.x + y * v.y;
        }
        T operator^(const Vector2D& v) const{ // cross product
            return x * v.y - y * v.x;
        }

        Vector2D operator+(const Vector2D& v) const{
            Vector2D vr;
            vr.x = x + v.x;
            vr.y = y + v.y;
            return vr;
        }
        Vector2D operator-(const Vector2D& v) const{
            Vector2D vr;
            vr.x = x - v.x;
            vr.y = y - v.y;
            return vr;
        }
        Vector2D operator*(const T& r) const{
            Vector2D vr;
            vr.x =  x * r;
            vr.y =  y * r;
            return vr;
        }
        Vector2D operator/(const T& r) const{
            Vector2D vr;
            vr.x = x / r;
            vr.y = y / r;
            return vr;
        }

        Vector2D& operator+=(const Vector2D& v){
            return *this = *this + v;
        }
        Vector2D& operator-=(const Vector2D& v){
            return *this = *this - v;
        }
        Vector2D& operator*=(const T& r){
            return *this = *this * r;
        }
        Vector2D& operator/=(const T& r){
            return *this = *this / r;
        }
        Vector2D& operator=(const Vector2D& v){
            x = v.x;
            y = v.y;
            return *this;
        }

        bool operator==(const Vector2D& v) const{
            return v.x == x && v.y == y;
        }
        bool operator!=(const Vector2D& v) const{
            return !(*this == v);
        }


        T norm()const{
            return sqrt(x * x + y * y);
        }
        void rotate(const T& theta){
            Vector2D vc = *this;
            x = vc.x * cos(theta) - vc.y * sin(theta);
            y = vc.x * sin(theta) + vc.y * cos(theta);
        }


        T x;
        T y;
};

template<typename T>
std::ostream& operator<<(std::ostream& out, Vector2D<T>& v){
    out << "(" << v.x << ";" << v.y << ")";
    return out;
}

#endif /* LIB_GEOMETRYTOOLS_VECTOR2D_H_ */
