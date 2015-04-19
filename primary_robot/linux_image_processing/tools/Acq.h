/*
 * Acq.h
 *
 *  Created on: 17 avr. 2015
 *      Author: ludo6431
 */

#ifndef TOOLS_ACQ_H_
#define TOOLS_ACQ_H_

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <Point2D.h>
#include <iostream>
#include <map>
#include <utility>
#include "ProjAcq.h"
#include <Plane3D.h>
#include <Vector2D.h>

class Cam;
class ProjAcq;

template<typename T> class Point2D;
template<typename T> class PosObj3D;
template<typename T> class Vector2D;

typedef enum ColorType {
    RGB,
    HSV
}eColorType;

class Acq {
protected:
    using matmap = std::map<eColorType, cv::Mat>;

    matmap matMap;
    Cam* cam;

public:
    Acq(cv::Mat mat, eColorType ctype, Cam* cam):cam(cam) {
        matMap.insert(std::pair<eColorType, cv::Mat>(ctype, mat));
    }
    virtual ~Acq() {
    }

    cv::Mat getMat(ColorType ctype = RGB);
    Cam* getCam();
    ProjAcq projectOnPlane(Plane3D<float> plan, Vector2D<int> size);
};

#endif /* TOOLS_ACQ_H_ */
