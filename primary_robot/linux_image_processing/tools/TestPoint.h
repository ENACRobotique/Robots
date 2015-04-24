/*
 * TestPoint.h
 *
 *  Created on: 17 avr. 2015
 *      Author: ludo6431
 */

#ifndef TOOLS_TESTPOINT_H_
#define TOOLS_TESTPOINT_H_

#include <opencv2/core/core.hpp>
#include <cmath>

class TestPoint {
public:
    cv::Mat pos; // (in cm, in playground reference frame)
    float hue;
    float weight;

    TestPoint(cv::Mat pos, float hue, float weight) :
            pos(pos), hue(hue), weight(weight) {
    }
    virtual ~TestPoint() {
    }

    float getCost(float mHue) const
            {
        float dHue = hue - mHue;

        if(dHue > 0.5) dHue -= 1.f;
        else if(dHue < -0.5) dHue += 1.f;

        return weight * std::abs(dHue);
    }

    float getHue() const
    {
        return hue;
    }

    /**
     * In centimeters, in playground reference frame
     */
    const cv::Mat getPos() const
    {
        return pos;
    }

    float getWeight() const
    {
        return weight;
    }
};

#endif /* TOOLS_TESTPOINT_H_ */
