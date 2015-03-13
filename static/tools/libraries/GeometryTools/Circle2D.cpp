/*
 * Circle2D.cpp
 *
 *  Created on: 8 mars 2015
 *      Author: Sébastien Malissard
 */

#include "Circle2D.h"

#include <cmath>

template<typename T>
Circle2D<T>::Circle2D(Point2D<T> _c, T _r) : c(_c), r(_r){
}

template<typename T>
Circle2D<T>::Circle2D(T _x, T _y, T _r) : c(_x,_y), r(_r){
}

template<typename T>
Circle2D<T>::~Circle2D(){
}

template<typename T>
std::pair<Point2D<T>, Point2D<T>>  Circle2D<T>::interCircle2Line(const Line2D<T>& l) const{
    T a, b, d, det;
    Point2D<T> p1, p2;
    Point2D<T> pt1;
    Point2D<T> pt2;

    //RET_IF_((l.a == 0) , ERR_BADPAR); FIXME
    //RET_IF_((l.b == 0) , ERR_BADPAR); FIXME

    if (l.a == 0) {
        a = l.b * l.b;
        b = -2 * c.x * l.b * l.b;
        d = l.b * l.b * (c.x * c.x + c.y * c.y - r * r) + 2 * l.b * l.c * c.x + l.c * l.c;
        if (((det = b * b - 4 * a * d) < 0) || (b == 0)) {
            return 0; //FIXME
        }else {
            p1.x = (-b - sqrt(det)) / (2 * a);
            p1.y = -l.c / l.b;
            p2.x = (-b + sqrt(det)) / (2 * a);
            p2.y = -l.c / l.b;
        }
    }else {
        a = (l.a * l.a + l.b * l.b);
        b = 2 * (l.b * l.c + l.a * l.b * c.x - l.a * l.a * c.y);
        d = l.a * l.a * (c.x * c.x + c.y * c.y - r * r) + 2 * l.a * l.c * c.x + l.c * l.c;
        if ((det = b * b - 4 * a * d) < 0) {
            return 0; //FIXME
        }else {
            p1.y = (-b - sqrt(det)) / (2 * a);
            p1.x = -(l.c + l.b * p1.y) / l.a;
            p2.y = (-b + sqrt(det)) / (2 * a);
            p2.x = -(l.c + l.b * p2.y) / l.a;
        }
    }

    return p1, p2;
}

template<typename T>
Point2D<T> Circle2D<T>::projPtOnCircle(Point2D<T>& p) const{
    Vector2D<T> v;
    Point2D<T> pt;
    T n;

    p.distPt2Pt(c, n);
   // if(n < 0.1)
   //     return ERR_BADPAR; //FIXME

    v.convPts2Vec(c, p);
    v.normVec(n);

    pt.x = c.x + v.x * r / n;
    pt.y = c.y + v.y * r / n;

    return pt;
}

template<typename T>
bool Circle2D<T>::checkPtOnArc(const Point2D<T>& p1, const Point2D<T>& p2, Point2D<T>& p) const{
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
