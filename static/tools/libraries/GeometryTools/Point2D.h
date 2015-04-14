/*
 * Point2D.h
 *
 *  Created on: 8 mars 2015
 *      Author: Sébastien Malissard
 */

#ifndef LIB_GEOMETRYTOOLS_POINT2D_H_
#define LIB_GEOMETRYTOOLS_POINT2D_H_

#include <iostream>
#include <cmath>

template<typename T>
class Vector2D;

template<typename T>
class Point2D {
    public:
        Point2D(): x(0), y(0){}
        Point2D(const Point2D& p): x(p.x), y(p.y){}
        Point2D(const T _x, const T _y) : x(_x), y(_y){}
        ~Point2D(){}

        Point2D operator+(const Vector2D<T>& v) const{
            return { x + v.x, y + v.y };
        }
        Point2D operator-(const Vector2D<T>& v) const{
            return { x - v.x, y - v.y };
        }
        Vector2D<T> operator-(const Point2D& o) const{
            return { x - o.x, y - o.y };
        }

        Point2D& operator+=(const Vector2D<T>& v){
            return *this = *this + v;
        }
        Point2D& operator-=(const Vector2D<T>& v){
            return *this = *this - v;
        }
        Point2D& operator=(const Point2D& p){
            x = p.x;
            y = p.y;
            return *this;
        }

        bool operator==(const Point2D& p) const{
            return p.x == x && p.y == y;
        }
        bool operator!=(const Point2D& p) const{
            return !(*this == p);
        }

        T distanceSqTo(const Point2D& p) const{
            return (p - *this).normSq();
        }
        T distanceTo(const Point2D& p) const{
            return (p - *this).norm();
        }

        void rotate(const T& a){
            Point2D pc = *this;
            x = pc.x*cos(a) - pc.y*sin(a);
            y = pc.x*sin(a) + pc.y*cos(a);
        }

        T x;
        T y;
};

template<typename T>
std::ostream& operator<<(std::ostream& out, Point2D<T>& p){
    out << "(" << p.x << ";" << p.y << ")";
    return out;
}

#endif /* LIB_GEOMETRYTOOLS_POIN2D_H_ */
