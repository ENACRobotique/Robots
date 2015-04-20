/*
 * Cam.h
 *
 *  Created on: 18 avr. 2015
 *      Author: ludo6431
 */

#ifndef TOOLS_CAM_H_
#define TOOLS_CAM_H_

#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types_c.h>
#include <Transform3D.h>
#include <Vector2D.h>

template<typename T> class Transform3D;
template<typename T> class Vector2D;

class Cam {
protected:
    cv::Mat matK_I2C = cv::Mat(3, 3, CV_32F);
    cv::Mat matK_C2I = cv::Mat(3, 3, CV_32F);
    cv::Mat matTransi_C2R = cv::Mat(4, 4, CV_32F);
    cv::Mat matTransi_R2C = cv::Mat(4, 4, CV_32F);

public:
    Cam(float f, Vector2D<float> size, Transform3D<float> posCam);

    cv::Mat getMatI2C() {
        return matK_I2C;
    }
    cv::Mat getMatC2I() {
        return matK_C2I;
    }
    cv::Mat getMatC2R() {
        return matTransi_C2R;
    }
    cv::Mat getMatR2C() {
        return matTransi_R2C;
    }
};

#endif /* TOOLS_CAM_H_ */
