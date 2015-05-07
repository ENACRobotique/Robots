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

/**
 * H (converted t0 0-1 range)
 * 0°-360° rouge
 * 60° jaune
 * 120° vert
 * 180° cyan
 * 240° bleu
 * 300° magenta
 *
 * S
 * 0 blanc
 * 1 couleur
 *
 * V
 * 0 noir
 * 1 couleur
 *
 */

class TestPoint {
public:
    cv::Mat pos; // (in cm, in playground reference frame)
    float hue, sat, val;
    float weight;

    TestPoint(float x, float y, float hue, float sat, float val, float weight) :
            pos((cv::Mat_<float>(2, 1) << x, y)), hue(hue), sat(sat), val(val), weight(weight) {
    }
    virtual ~TestPoint() {
    }

    float getCost(float mHue, float mSat, float mVal) const
            {
        float dVal = val - mVal; // dVal in range [-1;1]

        float dHue = hue - mHue;
        if (dHue > 0.5)
            dHue -= 1.f;
        else if (dHue < -0.5)
            dHue += 1.f;
        // dHue in range [-0.5;0.5]

        return weight * (val * std::abs(dHue) + (1.f - val) * 2.f * std::abs(dVal)); // encourages low val because black is a very good landmark this year :)
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
