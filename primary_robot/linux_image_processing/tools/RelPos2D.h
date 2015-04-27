/*
 * Position2D.h
 *
 *  Created on: 17 avr. 2015
 *      Author: ludo6431
 */

#ifndef TOOLS_RELPOS2D_H_
#define TOOLS_RELPOS2D_H_

#include <opencv2/core/core.hpp>
#include <Transform2D.h>
#include <Vector2D.h>

#ifdef USE_BOTNET
#include <messages-position.h>
#endif

template<typename T>
class RelPos2D {
    Vector2D<T> _v; // (cm x cm)
    T _theta; // (rad)

public:
    RelPos2D() :
            _theta(0) {
    }

    /**
     * x, y in centimeters
     * theta in radians
     */
    RelPos2D(T x, T y, T theta) :
            _v(x, y), _theta(theta) {
    }

    /**
     * v.x, v.y in centimeters
     * theta in radians
     */
    RelPos2D(Vector2D<T> v, T theta) :
            _v(v), _theta(theta) {
    }

    RelPos2D(cv::Mat v, T theta) :
            _v(v.at<T>(0), v.at<T>(1)), _theta(theta) {
    }

    T const& x() const {
        return _v.x;
    }
    T const& y() const {
        return _v.y;
    }
    T const& theta() const {
        return _theta;
    }

    RelPos2D operator+(const RelPos2D& v) const {
        return {_v + v._v, _theta + v._theta};
    }
    RelPos2D operator-(const RelPos2D& v) const {
        return {_v - v._v, _theta - v._theta};
    }
    RelPos2D operator*(const T& r) const {
        return {_v*r, _theta*r};
    }
    RelPos2D operator/(const T& r) const {
        return {_v/r, _theta/r};
    }

    RelPos2D& operator+=(const RelPos2D& v) {
        return *this = *this + v;
    }
    RelPos2D& operator-=(const RelPos2D& v) {
        return *this = *this - v;
    }
    RelPos2D& operator*=(const T& r) {
        return *this = *this * r;
    }
    RelPos2D& operator/=(const T& r) {
        return *this = *this / r;
    }

    Transform2D<T> getTransform() const {
        return Transform2D<T>(_v.x, _v.y, _theta);
    }
};

template<typename T>
RelPos2D<T> operator*(const T& n, const RelPos2D<T>& v) {
    return v * n;
}

#endif /* TOOLS_ABSPOS2D_H_ */
