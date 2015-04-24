/*
 * Point2D.h
 *
 *  Created on: 8 mars 2015
 *      Author: Sébastien Malissard
 */

#ifndef LIB_GEOMETRYTOOLS_POINT2D_H_
#define LIB_GEOMETRYTOOLS_POINT2D_H_

#include <Vector2D.h>
#include <cmath>
#include <iostream>

#ifdef USE_OPENCV
#include <opencv2/core/core.hpp>
#endif

template<typename T>
class Point2D {
public:
    T x, y;

    Point2D() :
            x(0), y(0) {
    }
    Point2D(const Point2D& p) :
            x(p.x), y(p.y) {
    }
    Point2D(const T _x, const T _y) :
            x(_x), y(_y) {
    }
#ifdef USE_OPENCV
    Point2D(cv::Mat m) :
            x(m.at<T>(0)), y(m.at<T>(1)) {
    }
#endif
    ~Point2D() {
    }

#ifdef USE_OPENCV
    cv::Mat toCv() const {
        return (cv::Mat_<T>(2, 1) << x, y);
    }
#endif

    Point2D operator+(const Vector2D<T>& v) const {
        return {x + v.x, y + v.y};
    }
    Point2D operator-(const Vector2D<T>& v) const {
        return {x - v.x, y - v.y};
    }
    Vector2D<T> operator-(const Point2D& o) const {
        return {x - o.x, y - o.y};
    }

    Point2D& operator+=(const Vector2D<T>& v) {
        return *this = *this + v;
    }
    Point2D& operator-=(const Vector2D<T>& v) {
        return *this = *this - v;
    }
    Point2D& operator=(const Point2D& p) {
        x = p.x;
        y = p.y;
        return *this;
    }

    bool operator==(const Point2D& p) const {
        return p.x == x && p.y == y;
    }
    bool operator!=(const Point2D& p) const {
        return !(*this == p);
    }

    T distanceSqTo(const Point2D& p) const {
        return (p - *this).normSq();
    }
    T distanceTo(const Point2D& p) const {
        return (p - *this).norm();
    }

    Point2D& rotate(const T& a) {
        Point2D pc = *this;
        x = pc.x * cos(a) - pc.y * sin(a);
        y = pc.x * sin(a) + pc.y * cos(a);
        return *this;
    }
};

template<typename T>
std::ostream& operator<<(std::ostream& out, const Point2D<T>& p) {
    out << "(" << p.x << ";" << p.y << ")";
    return out;
}

#endif /* LIB_GEOMETRYTOOLS_POIN2D_H_ */
