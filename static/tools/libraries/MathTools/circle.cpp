/*
 * circle.cpp
 *
 *  Created on: 8 mars 2015
 *      Author: Sébastien Malissard
 */

#include <circle.h>

#include <cmath>

Circle::Circle(Point _c, float _r) : c(_c), r(_r){
}

Circle::Circle(float _x, float _y, float _r) : c(_x,_y), r(_r){
}

Circle::~Circle(){
}

ERROR Circle::interCircle2Line(const Line& l, int& nb, Point& pt1, Point& pt2) const{
    float a, b, d, det;
    Point p1, p2;

    RET_IF_((l.a == 0) , ERR_BADPAR);
    RET_IF_((l.b == 0) , ERR_BADPAR);

    if (l.a == 0) {
        a = l.b * l.b;
        b = -2 * c.x * l.b * l.b;
        d = l.b * l.b * (c.x * c.x + c.y * c.y - r * r) + 2 * l.b * l.c * c.x + l.c * l.c;
        if (((det = b * b - 4 * a * d) < 0) || (b == 0)) {
            nb = 0;
            return 0;
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
            nb = 0;
            return 0;
        }else {
            p1.y = (-b - sqrt(det)) / (2 * a);
            p1.x = -(l.c + l.b * p1.y) / l.a;
            p2.y = (-b + sqrt(det)) / (2 * a);
            p2.x = -(l.c + l.b * p2.y) / l.a;
        }
    }

    if ((p1.x == p2.x) && (p1.y == p2.y))
        nb = 1;
    else
        nb = 2;

    pt1 = p1;
    pt2 = p2;


    return 0;
}

ERROR Circle::projPtOnCircle(Point& p) const{
    Vector v;
    float n;

    p.distPt2Pt(c, n);
    if(n < 0.1)
        return ERR_BADPAR;

    v.convPts2Vec(c, p);
    v.normVec(n);

    p.x = c.x + v.x * r / n;
    p.y = c.y + v.y * r / n;

    return 0;
}

ERROR Circle::checkPtOnArc(const Point& p1, const Point& p2, Point& p, bool& ret) const{
    float theta1, theta2, theta;

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
            ret = false;
            return 0;
        }
    }
    else {
        if ((theta < theta1) || (theta > theta2)) {
            ret = false;
            return 0;
        }
    }

    ret = true;
    return 0;
}
