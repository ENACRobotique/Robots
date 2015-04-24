/*
 * Position2D.h
 *
 *  Created on: 17 avr. 2015
 *      Author: ludo6431
 */

#ifndef TOOLS_POSITION2D_H_
#define TOOLS_POSITION2D_H_

#include <Point2D.h>
#include <Transform2D.h>

#ifdef USE_BOTNET
#include <messages-position.h>
#endif

template<typename T>
class Position2D {
    cv::Mat p; // (cm x cm)
    T _theta; // (rad)

public:
    /**
     * x, y in centimeters
     * theta in radians
     */
    Position2D(T x, T y, T theta) :
            p((cv::Mat_<T>(2, 1) << x, y)), _theta(theta) {
    }

    /**
     * p.x, p.y in centimeters
     * theta in radians
     */
    Position2D(Point2D<T> p, T theta) :
            p(p.toCv()), _theta(theta) {
    }

    Position2D(cv::Mat p, T theta) :
        p(p), _theta(theta) {
    }

#ifdef USE_BOTNET
    Position2D(const s2DPosAtt& p) :
            p(p.x, p.y), _theta(p.theta) {
    }
#endif

    T x() const {
        return p.at<T>(0);
    }

    T y() const {
        return p.at<T>(1);
    }

    T theta() const {
        return _theta;
    }

    cv::Mat getLinPos() const {
        return p;
    }

    Transform2D<T> getTransform() const {
        return Transform2D<T>(p.at<T>(0), p.at<T>(1), _theta);
    }
};

#endif /* TOOLS_POSITION2D_H_ */
