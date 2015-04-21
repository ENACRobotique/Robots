/*
 * Acq.h
 *
 *  Created on: 17 avr. 2015
 *      Author: ludo6431
 */

#ifndef TOOLS_ACQ_H_
#define TOOLS_ACQ_H_

#include <opencv2/core/core.hpp>
#include <Plane3D.h>
#include <tools/Image.h>
#include <tools/ProjAcq.h>
#include <map>
#include <utility>

class Cam;
class ProjAcq;

class Acq: public Image {
protected:
    Cam const* cam;

public:
    Acq(cv::Mat mat, eColorType ctype, Cam const* cam) :
            cam(cam) {
        matMap.insert(std::pair<eColorType, cv::Mat>(ctype, mat));
    }
    virtual ~Acq() {
    }

    Cam const* getCam() const {
        return cam;
    }

    ProjAcq projectOnPlane(const Plane3D<float>& plan, cv::Size size) {
        return ProjAcq(size, this, plan);
    }
};

#endif /* TOOLS_ACQ_H_ */
