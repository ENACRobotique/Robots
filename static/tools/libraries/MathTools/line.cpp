/*
 * line.cpp
 *
 *  Created on: 8 mars 2015
 *      Author: Sébastien Malissard
 */

#include "line.h"

Line::Line(float _a, float _b, float _c) : a(_a), b(_b), c(_c), norm(false) {
}

Line::~Line() {
}

ERROR Line::normLine() {

    if (!norm) {
        Vector nv(-b, a);
        float n;

        nv.normVec(n);

        RET_IF_(n == 0, ERR_DIV0);

        a = a / n;
        b = b / n;
        c = c / n;

        norm = true;
    }

    return 0;
}

ERROR Line::interLine2Line(const Line& l, int& nb, Point& pt) const{
    float det;

    if (!(det = a * l.b - b * l.a)) { //parallel
        nb = 0;
    }
    else {
        pt.x = (1 / det) * (b * l.c - c * l.b);
        pt.y = (1 / det) * (c * l.a - a * l.c);
        nb = 1;
    }

    return 0;
}

ERROR Line::convPts2Line(const Point& p1, const Point& p2, bool& _norm){

    norm = _norm;

    if (!norm) {
        a = p2.y - p1.y;
        b = p1.x - p2.x;
    }
    else {
        float n;
        p1.distPt2Pt(p2, n);

        a = (p2.y - p1.y) / n;
        b = (p1.x - p2.x) / n;
    }

    c = -a * p1.x - b * p1.y;

    return 0;
}

ERROR Line::convVecPt2Line(const Vector& v, const Point& p, bool& _norm){

    norm = _norm;

    if (!norm) {
        a = v.y;
        b = -v.x;
    }
    else {
        float n;
        v.normVec(n);

        a = v.y / n;
        b = -v.x / n;
    }

    c = -a * p.x - b * p.y;

    return 0;
}


ERROR Line::distPt2Line(const Point& p, float& d, Point& h){

    this->normLine();

    d = a * p.x + b * p.y + c;

    h.x =  b * (b * p.x - a * p.y) - a * c;
    h.y = -a * (b * p.x - a * p.y) - b * c;


    return 0;
}

ERROR Line::symPtprLine(Point& p){
    Point pp(0,0), pc(p);
    float d;

    this->distPt2Line(p, d, pp);

    p.x = pc.x - 2 * (pc.x - pp.x);
    p.y = pc.y - 2 * (pc.y - pp.y);

    return 0;
}
