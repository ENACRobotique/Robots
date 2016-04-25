/*
 * Cam.h
 *
 *  Created on: 18 avr. 2015
 *      Author: ludo6431
 */

#ifndef TOOLS_CAM_H_
#define TOOLS_CAM_H_

#include <opencv2/core/core.hpp>
#include <opencv2/core/types_c.h>
#include <Point2D.h>
#include <Transform3D.h>
#include <cmath>

class Cam {
protected:
    cv::Mat matK_I2C = cv::Mat(3, 3, CV_32F);
    cv::Mat matK_C2I = cv::Mat(3, 3, CV_32F);
    cv::Mat matTransi_C2R = cv::Mat(4, 4, CV_32F);
    cv::Mat matTransi_R2C = cv::Mat(4, 4, CV_32F);
    Transform3D<float> rob2cam;
    cv::Size2f aperAngle;
    cv::Size size;

public:
    /**
     * f (in pixels)
     * size (in pixels)
     */
    // Constructor
    Cam(float f, cv::Size size, const Transform3D<float>& rob2cam) :
            rob2cam(rob2cam), size(size) {
        aperAngle = cv::Size2f(
                2. * atan2f(size.width / 2, f),
                2. * atan2f(size.height / 2, f));

        // Construct transition matrix from image to camera
        matK_C2I = (cv::Mat_<float>(3, 3) <<
                f, 0, (size.width - 1.f) / 2.f,
                0, f, (size.height - 1.f) / 2.f,
                0, 0, 1);

        // Construct the transition matrix from camera to image
        matK_I2C = matK_C2I.inv();

        // 3D transformation from robot reference frame to cam one
        matTransi_R2C = rob2cam.getMatrix();

        // 3D transformation from cam reference frame to robot one
        matTransi_C2R = rob2cam.getReverse().getMatrix();
    }

    // Accessors
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
            matK_C2I.at<float>(0, 0),
            matK_C2I.at<float>(1, 1)};
    }
    cv::Size getSize() const {
        return size;
    }
    const Transform3D<float>& getRob2Cam() const {
        return rob2cam;
    }
    const cv::Size2f& getAperAngle() const {
        return aperAngle;
    }

    cv::Mat getTopLeft() const {
        return (cv::Mat_<float>(2, 1) << 0, 0);
    }
    cv::Mat getTopRight() const {
        return (cv::Mat_<float>(2, 1) << size.width - 1, 0);
    }
    cv::Mat getBottomRight() const {
        return (cv::Mat_<float>(2, 1) << size.width - 1, size.height - 1);
    }
    cv::Mat getBottomLeft() const {
        return (cv::Mat_<float>(2, 1) << 0, size.height - 1);
    }
    cv::Mat getCenter() const {
        return (cv::Mat_<float>(2, 1) << (size.width - 1.f) / 2.f, (size.height - 1.f) / 2.f);
    }
};

#endif /* TOOLS_CAM_H_ */
