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
    Vector2D<T> _camDir;

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
     * x, y in centimeters
     * theta in radians
     */
    RelPos2D(T x, T y, T theta, Vector2D<T> const& camDir) :
            _v(x, y), _theta(theta), _camDir(camDir) {
    }

    /**
     * v.x, v.y in centimeters
     * theta in radians
     */
    RelPos2D(Vector2D<T> v, T theta) :
            _v(v), _theta(theta) {
    }

    /**
     * v.x, v.y in centimeters
     * theta in radians
     */
    RelPos2D(Vector2D<T> v, T theta, Vector2D<T> const& camDir) :
            _v(v), _theta(theta), _camDir(camDir) {
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
    Vector2D<T> const& v() const {
        return _v;
    }
    T const& theta() const {
        return _theta;
    }
    Vector2D<T> const& camDir() const {
        return _camDir;
    }

    RelPos2D operator+(const RelPos2D& v) const {
        return {_v + v._v, _theta + v._theta, v._camDir};
    }
    RelPos2D operator-(const RelPos2D& v) const {
        return {_v - v._v, _theta - v._theta, v._camDir};
    }
    RelPos2D operator*(const T& r) const {
        return {_v*r, _theta*r, _camDir};
    }
    RelPos2D operator/(const T& r) const {
        return {_v/r, _theta/r, _camDir};
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
    RelPos2D& operator=(const RelPos2D& v) {
        _v = v._v;
        _theta = v._theta;
        _camDir = v._camDir;
        return *this;
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
