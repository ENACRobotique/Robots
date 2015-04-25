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
    cv::Mat matVectOcPx_R = cv::Mat(4,3, CV_32F);
    Transform3D<float> rob2cam;
    cv::Size2f aperAngle;
    cv::Size size;

public:
    /**
     * f (in pixels)
     * size (in pixels)
     */
    Cam(float f, cv::Size size, const Transform3D<float>& rob2cam) :
            rob2cam(rob2cam),size(size) {
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

        // Set a matrix A to stock the constant values to compute the vector OcPx_R (OrigCam2Pixel in the robot)
        for(int i=0; i<3; i++){
            for(int j=0; j<3; j++){
                if(j == 0)
                    matVectOcPx_R.at<float>(i,j) = matTransi_C2R.at<float>(i,0)*matK_I2C.at<float>(0,0);
                if(j == 1)
                    matVectOcPx_R.at<float>(i,j) = matTransi_C2R.at<float>(i,1)*matK_I2C.at<float>(1,1);  // Useful if focal_x = focal_y
                if(j==2)
                    matVectOcPx_R.at<float>(i,j) = matTransi_C2R.at<float>(i,3) + matTransi_C2R.at<float>(i,2) +
                                                   matTransi_C2R.at<float>(i,0)*matK_I2C.at<float>(0,2) +
                                                   matTransi_C2R.at<float>(i,1)*matK_I2C.at<float>(1,2);
            }
        }
        matVectOcPx_R.at<float>(3,0) = 1;
        matVectOcPx_R.at<float>(3,1) = matVectOcPx_R.at<float>(3,2) = 0;
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
    /*
     * Description: Compute the vector origin camera to a pixel(u,v) (image cam1) in robot
     *      OcPx_R = matVectOcPx_R * Px(u,v,1)
     */
    cv::Mat getOcPx_R(const cv::Mat px_in) const {
        cv::Mat px(px_in);
        if(px_in.cols != 1  &&  px_in.rows < 2){ // Not enough information
            std::cout<<"getOcPx_R(): (u,v) are missing in the input matrix"<<std::endl;
            return cv::Mat::zeros(px_in.rows,px_in.cols, CV_8UC1);
        }
        else if(px_in.cols != 1  &&  px_in.rows != 4){ // it has u and v but the size is not matched
            std::cout<<"getOcPx_R(): size of the input matrix changed"<<std::endl;
            px.create(4,1, CV_32F);
        }
        cv::Mat OcPx_R = cv::Mat(4, 1, CV_32F);
        for(int i=0; i<4; i++){
            OcPx_R = matVectOcPx_R.at<float>(i,0)*px.at<float>(0) +
                     matVectOcPx_R.at<float>(i,1)*px.at<float>(1) +
                     matVectOcPx_R.at<float>(i,2);
        }
        return OcPx_R;
    }
};

#endif /* TOOLS_CAM_H_ */
