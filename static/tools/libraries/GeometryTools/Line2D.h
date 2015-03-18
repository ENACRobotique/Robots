/*
 * Line2D.h
 *
 *  Created on: 8 mars 2015
 *      Author: Sébastien Malissard
 */

#ifndef LIB_GEOMETRYTOOLS_LINE2D_H_
#define LIB_GEOMETRYTOOLS_LINE2D_H_

#include <iostream>

template<typename T>
class Point2D;
template<typename T>
class Vector2D;

template<typename T>
class Line2D {
    public:
        Line2D() : a(0), b(0), c(0), norm(false){}
        Line2D(const Line2D& l) : a(l.a), b(l.b), c(l.c), norm(l.norm){}
        Line2D(const T _a, const T _b, const T _c) : a(_a), b(_b), c(_c), norm(false){}
        Line2D(const Vector2D<T>& v) : a(v.y), b(-v.x), c(0), norm(false){}
        Line2D(const Point2D<T>& p, const Vector2D<T>& v) : a(v.y), b(-v.x), c(c = -a * p.x - b * p.y), norm(false){}
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
            l.normLine();
            normLine();
            return l.a == a && l.b == b && l.c == c;
        }
        bool operator!=(Line2D& l){
            return !(*this == l);
        }

        void normLine(){
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
        T distance(const Point2D<T>& p){
            normLine();
            return a * p.x + b * p.y + c;
        }
        Point2D<T> projecte(const Point2D<T>& p){
            Point2D<T> pr;

            this->normLine();
            pr.x =  b * (b * p.x - a * p.y) - a * c;
            pr.y = -a * (b * p.x - a * p.y) - b * c;

            return pr;
        }
        Point2D<T> symetry(Point2D<T>& p){
            Point2D<T> pp(0,0), pc(p), pr;

            pp = this->projecte(p);
            pr.x = pc.x - 2 * (pc.x - pp.x);
            pr.y = pc.y - 2 * (pc.y - pp.y);

            return pr;
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
