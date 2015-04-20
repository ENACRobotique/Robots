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
#include <opencv2/core/operations.hpp>
#include <opencv2/core/types_c.h>
#include <Transform3D.h>
#include <Vector2D.h>
#include <cmath>

template<typename T> class Transform3D;
template<typename T> class Vector2D;

class Cam {
protected:
    cv::Mat matK_I2C = cv::Mat(3, 3, CV_32F);
    cv::Mat matK_C2I = cv::Mat(3, 3, CV_32F);
    cv::Mat matTransi_C2R = cv::Mat(4, 4, CV_32F);
    cv::Mat matTransi_R2C = cv::Mat(4, 4, CV_32F);
    Transform3D<float> rob2cam;
    cv::Size2f aperAngle;

public:
    /**
     * f (in pixels)
     * size (in pixels)
     */
    Cam(float f, cv::Size size, const Transform3D<float>& rob2cam) :
            rob2cam(rob2cam) {
        aperAngle = cv::Size2f(
                2. * atan2f(size.width / 2, f),
                2. * atan2f(size.height / 2, f));

        // Construct transition matrix from image to camera
        matK_C2I = (cv::Mat_<float>(3, 3) <<
                f, 0, (size.width - 1) / 2,
                0, f, (size.height - 1) / 2,
                0, 0, 1);

        // Construct the transition matrix from camera to image
        matK_C2I = matK_I2C.inv();

        // 3D transformation from robot reference frame to cam one
        matTransi_R2C = rob2cam.getMatrix();

        // 3D transformation from cam reference frame to robot one
        matTransi_C2R = rob2cam.getReverse().getMatrix();
    }

    const cv::Mat& getMatI2C() const {
        return matK_I2C;
    }
    const cv::Mat& getMatC2I() const {
        return matK_C2I;
    }
    const cv::Mat& getMatC2R() const {
        return matTransi_C2R;
    }
    const cv::Mat& getMatR2C() const {
        return matTransi_R2C;
    }
    cv::Size2f getFocal() const {
        return {
            this->matK_C2I.at<float>(0, 0),
            this->matK_C2I.at<float>(1, 1)};
    }
    cv::Size getSize() const {
        return {
            (this->matK_C2I.at<float>(0,2)*2) + 1,
            (this->matK_C2I.at<float>(1,2)*2) + 1};
    }
    const Transform3D<float>& getRob2Cam() const {
        return rob2cam;
    }
    const cv::Size2f& getAperAngle() const {
        return aperAngle;
    }
};

#endif /* TOOLS_CAM_H_ */