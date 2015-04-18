/*
 * TestPoint.h
 *
 *  Created on: 17 avr. 2015
 *      Author: ludo6431
 */

#ifndef TOOLS_TESTPOINT_H_
#define TOOLS_TESTPOINT_H_

#include <Point2D.h>

class TestPoint {
    Point2D<float> pos; // (in cm, in playground reference frame)
    float hue;
    float weight;

public:
    TestPoint(const Point2D<float>& pos, float hue, float weight) :
            pos(pos), hue(hue), weight(weight) {
    }
    virtual ~TestPoint() {
    }

    float getCost(float mHue) const
            {
        return weight * std::abs(hue - mHue);
    }

    float getHue() const
    {
        return hue;
    }

    /**
     * In centimeters, in playground reference frame
     */
    const Point2D<float>& getPos() const
    {
        return pos;
    }

    float getWeight() const
    {
        return weight;
    }
};

#endif /* TOOLS_TESTPOINT_H_ */
