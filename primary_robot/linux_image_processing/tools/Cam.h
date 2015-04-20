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
#include <opencv2/imgproc/imgproc.hpp>
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
    Transform3D<float> pos;
    float aperAngle;

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
    float getFocal() {
        return this->matK_C2I.at<float>(0,0);
    }
    Vector2D<int> getSize() {
        Vector2D<int> size;
        size.x = (this->matK_C2I.at<float>(0,2)*2) + 1;
        size.y = (this->matK_C2I.at<float>(1,2)*2) + 1;

        return size;
    }
    Transform3D<float> getTranform() {
        return pos;
    }
    float getAperAngle() {
        return aperAngle;
    }
};

#endif /* TOOLS_CAM_H_ */
