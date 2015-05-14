/*
 * Line2D.h
 *
 *  Created on: 8 mars 2015
 *      Author: Sébastien Malissard
 */

#ifndef LIB_GEOMETRYTOOLS_LINE2D_H_
#define LIB_GEOMETRYTOOLS_LINE2D_H_

#include <iostream>

#include "Vector2D.h"

template<typename T>
class Point2D;

template<typename T>
class Line2D {
    public:
        Line2D() : a(0), b(0), c(0), norm(false){}
        Line2D(const Line2D& l) : a(l.a), b(l.b), c(l.c), norm(l.norm){}
        Line2D(const T _a, const T _b, const T _c) : a(_a), b(_b), c(_c), norm(false){}
        Line2D(const Vector2D<T>& v) : a(v.y), b(-v.x), c(0), norm(false){}
        Line2D(const Point2D<T>& p, const Vector2D<T>& v) : a(v.y), b(-v.x), c(-a * p.x - b * p.y), norm(false){}
        Line2D(const Point2D<T>& p1, const Point2D<T>& p2) : a(p2.y - p1.y), b(p1.x - p2.x), c(-a * p1.x - b * p1.y), norm(false){}
        ~Line2D(){};

        Line2D& operator=(const Line2D& l){
            a = l.a;
            b = l.b;
            c = l.c;
            norm = l.norm;
            return *this;
        }

        bool operator==(Line2D& l){
            l.normalize();
            normalize();
            return l.a == a && l.b == b && l.c == c;
        }
        bool operator!=(Line2D& l){
            return !(*this == l);
        }

        void normalize(){
            if (!norm) {
                Vector2D<T> nv(-b, a);
                T n = nv.norm();
                a = a / n;
                b = b / n;
                c = c / n;
                norm = true;
            };
        }
        Point2D<T> intersection(const Line2D& l) const{
            T det;

            if (!(det = a * l.b - b * l.a)) { //parallel
                return 0; // FIXME
            }
            Point2D<T> p;
            p.x = (1 / det) * (b * l.c - c * l.b);
            p.y = (1 / det) * (c * l.a - a * l.c);

            return p;
        }
        T distanceTo(const Point2D<T>& p){
            normalize();
            return a * p.x + b * p.y + c;
        }
        Point2D<T> project(const Point2D<T>& p){
            Point2D<T> pr;

            normalize();
            pr.x =  b * (b * p.x - a * p.y) - a * c;
            pr.y = -a * (b * p.x - a * p.y) - b * c;

            return pr;
        }
        Point2D<T> symetry(const Point2D<T>& p){
            return p + T(2) * (project(p) - p);
        }

        // 2D line (ax+by+c=0)
        T a;
        T b;
        T c;
        bool norm;
};

template<typename T>
std::ostream& operator<<(std::ostream& out, Line2D<T>& l){
    out << l.a << "*x + " << l.b << "*y + "<< l.c << " = 0";
    return out;
}

#endif /* LIB_GEOMETRYTOOLS_LINE2D_H_ */
