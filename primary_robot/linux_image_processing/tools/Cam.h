/*
 * Cam.h
 *
 *  Created on: 18 avr. 2015
 *      Author: ludo6431
 */

#ifndef TOOLS_CAM_H_
#define TOOLS_CAM_H_

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ConfObj3D.h>
#include <Vector2D.h>

template<typename T> class ConfObj3D;
template<typename T> class Vector2D;



class Cam {
protected:
    cv::Mat K_I2C = cv::Mat(3,3,CV_32F);
    cv::Mat K_C2I = cv::Mat(3,3,CV_32F);
    cv::Mat Transi_C2R = cv::Mat(4,4,CV_32F);
    cv::Mat Transi_R2C = cv::Mat(4,4,CV_32F);
    ConfObj3D<float> conf;
    float aperAngl;

public:
    Cam(float f, Vector2D<float> size, ConfObj3D<float> confCam);

    cv::Mat getMatI2C();
    cv::Mat getMatC2I();
    cv::Mat getMatC2R();
    cv::Mat getMatR2C();
    float getFocal();
    Vector2D<int> getSize();
    ConfObj3D<float> getConf();
    float getAperAngle();

};

#endif /* TOOLS_CAM_H_ */
