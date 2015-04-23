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
public:
    Point2D<T> p; // (cm x cm)
    T theta; // (rad)

    /**
     * x, y in centimeters
     * theta in radians
     */
    Position2D(T x, T y, T theta) :
            p(x, y), theta(theta) {
    }

    /**
     * p.x, p.y in centimeters
     * theta in radians
     */
    Position2D(Point2D<T> p, T theta) :
            p(p), theta(theta) {
    }

#ifdef USE_BOTNET
    Position2D(const s2DPosAtt& p) :
            p(p.x, p.y), theta(p.theta) {
    }
#endif

    Transform2D<T> getTransform() const {
        return Transform2D<T>(p.x, p.y, theta);
    }
};

#endif /* TOOLS_POSITION2D_H_ */
