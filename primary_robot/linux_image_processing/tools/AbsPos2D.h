/*
 * Position2D.h
 *
 *  Created on: 17 avr. 2015
 *      Author: ludo6431
 */

#ifndef TOOLS_ABSPOS2D_H_
#define TOOLS_ABSPOS2D_H_

#include <opencv2/core/core.hpp>
#include <Point2D.h>
#include <tools/RelPos2D.h>
#include <Transform2D.h>
#include <cmath>
#include <iostream>

#ifdef USE_BOTNET
#include <messages-position.h>
#endif

template<typename T>
class AbsPos2D;

template<typename T>
std::ostream& operator<<(std::ostream& out, const AbsPos2D<T>& p);

template<typename T>
class AbsPos2D {
    Point2D<T> _p; // (cm x cm)
    T _theta; // (rad)

public:
    AbsPos2D() :
            _theta(0) {
    }

    /**
     * x, y in centimeters
     * theta in radians
     */
    AbsPos2D(T x, T y, T theta) :
            _p(x, y), _theta(theta) {
    }

    /**
     * p.x, p.y in centimeters
     * theta in radians
     */
    AbsPos2D(Point2D<T> p, T theta) :
            _p(p), _theta(theta) {
    }

    AbsPos2D(cv::Mat p, T theta) :
            _p(p.at<T>(0), p.at<T>(1)), _theta(theta) {
    }

#ifdef USE_BOTNET
    AbsPos2D(const s2DPosAtt& p) :
            _p(p.x, p.y), _theta(p.theta) {
    }
#endif

    T const& x() const {
        return _p.x;
    }

    T const& y() const {
        return _p.y;
    }

    T const& theta() const {
        return _theta;
    }

    AbsPos2D operator+(const RelPos2D<T>& v) const {
        return {_p.x + v.x(), _p.y + v.y(), _theta + v.theta()};
    }
    RelPos2D<T> operator-(const AbsPos2D& p) const {
        return {_p.x - p._p.x, _p.y - p._p.y, _theta - p._theta};
    }

    Transform2D<T> getTransform() const {
        return Transform2D<T>(_p.x, _p.y, _theta);
    }

    friend std::ostream& operator<<<T>(std::ostream& out, const AbsPos2D& p);
};

template<typename T>
std::ostream& operator<<(std::ostream& out, const AbsPos2D<T>& p) {
    out << "(" << p.x() << " cm;" << p.y() << " cm;" << p.theta() * T(180) / T(M_PI) << " deg)";
    return out;
}

#endif /* TOOLS_ABSPOS2D_H_ */
