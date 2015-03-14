/*
 * Circle2D.h
 *
 *  Created on: 8 mars 2015
 *      Author: Sébastien Malissard
 */

#ifndef LIB_GEOMETRYTOOLS_CIRCLE2D_H_
#define LIB_GEOMETRYTOOLS_CIRCLE2D_H_

#include <iostream>
#include <cmath>

template<typename T>
class Point2D;
template<typename T>
class Line2D;

template<typename T>
class Circle2D {
    public:
        Circle2D() : c(0, 0), r(0){}
        Circle2D(Point2D<T> _c, T _r) : c(_c), r(_r){}
        Circle2D(T _x, T _y, T _r) : c(_x,_y), r(_r){}
        ~Circle2D(){}

        Circle2D& operator=(const Circle2D& c){
            c = c.c;
            r = c.r;
            return *this;
        }

        bool operator==(const Circle2D& c) const{
            return c.c == c && c.r == r;
        }
        bool operator!=(const Circle2D& c) const{
            return !(*this == c);
        }

        std::pair<Point2D<T>, Point2D<T>> intersection(const Line2D<T>& l) const{
            T a, b, d, det;
            Point2D<T> p1, p2;

            if(!l.a){
                a = l.b * l.b;
                b = -2 * c.x * l.b * l.b;
                d = l.b * l.b * (c.x * c.x + c.y * c.y - r * r) + 2 * l.b * l.c * c.x + l.c * l.c;
                if (((det = b * b - 4 * a * d) < 0) || (b == 0)){
                    return 0; //FIXME
                }else{
                    p1.x = (-b - sqrt(det)) / (2 * a);
                    p1.y = -l.c / l.b;
                    p2.x = (-b + sqrt(det)) / (2 * a);
                    p2.y = -l.c / l.b;
                }
            }else{
                a = (l.a * l.a + l.b * l.b);
                b = 2 * (l.b * l.c + l.a * l.b * c.x - l.a * l.a * c.y);
                d = l.a * l.a * (c.x * c.x + c.y * c.y - r * r) + 2 * l.a * l.c * c.x + l.c * l.c;
                if ((det = b * b - 4 * a * d) < 0){
                    return 0; //FIXME
                }else{
                    p1.y = (-b - sqrt(det)) / (2 * a);
                    p1.x = -(l.c + l.b * p1.y) / l.a;
                    p2.y = (-b + sqrt(det)) / (2 * a);
                    p2.x = -(l.c + l.b * p2.y) / l.a;
                }
            }

            return p1, p2;
        }
        Point2D<T> projecte(Point2D<T>& p) const{
            Vector2D<T> v;
            Point2D<T> pt;
            T n;

            v.convPts2Vec(c, p);
            v.normVec(n);
            pt.x = c.x + v.x * r / n;
            pt.y = c.y + v.y * r / n;

            return pt;
        }
        bool checkPointOnArc(const Point2D<T>& p1, const Point2D<T>& p2, Point2D<T>& p) const{ // p1 and p2 is an angle
            T theta1, theta2, theta;

            if ((theta1 = atan2(p1.y - c.y, p1.x - c.x)) < 0) {
                theta1 += 2 * M_PI;
            }
            if ((theta2 = atan2(p2.y - c.y, p2.x - c.x)) < 0) {
                theta2 += 2 * M_PI;
            }
            if ((theta = atan2(p.y - c.y, p.x - c.x)) < 0) {
                theta += 2 * M_PI;
            }

            if (theta2 < theta1) {
                if ((theta < theta1) && (theta > theta2)) {
                    return false;
                }
            }
            else {
                if ((theta < theta1) || (theta > theta2)) {
                    return false;
                }
            }

            return true;
        }
        T surface() const{
            return M_PI * r * r;
        }
        T perimeter() const{
            return 2 * M_PI * r;
        }

        Point2D<T> c;
        T r;
};

template<typename T>
std::ostream& operator<<(std::ostream& out, Circle2D<T>& c){
    out << "(" << c.c << ";" << c.r << ")";
    return out;
}

#endif /* LIB_GEOMETRYTOOLS_CIRCLE2D_H_ */
