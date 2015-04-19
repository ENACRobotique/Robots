#include "Cam.h"

Cam::Cam(float f, Vector2D<float> size, ConfObj3D<float> confCam){
    conf = confCam;
    aperAngl = atan2(size.y/2, f);
    // Construct transition matrix from image to camera
    K_C2I = (cv::Mat_<float>(3,3) << f, 0, (size.x-1)/2,
                                     0, f, (size.y-1)/2,
                                     0, 0,  1);

    // Construct the transition matrix from camera to image
    K_C2I = K_I2C.inv();

    // construct the rotation matrix from robot to camera
    float crx = cos(conf.rx);
    float srx = sin(conf.rx);
    float cry = cos(conf.ry);
    float sry = sin(conf.ry);
    float crz = cos(conf.rz);
    float srz = sin(conf.rz);
    cv::Mat rotX_R2C = (cv::Mat_<float>(3,3) <<   1,   0,     0,
                                                  0,  crx,  srx,
                                                  0, -srx,  crx);
    cv::Mat rotY_R2C = (cv::Mat_<float>(3,3) << cry,    0, -sry,
                                                  0,    1,    0,
                                                sry,    0,  cry);
    cv::Mat rotZ_R2C = (cv::Mat_<float>(3,3) << crz,  srz,    0,
                                               -srz,  crz,    0,
                                                  0,    0,    1);
    cv::Mat rot_R2C = rotX_R2C*rotY_R2C*rotZ_R2C;

    // Construct the rotation matrix from camera to robot
    cv::Mat rot_C2R = rot_R2C.inv();

    // Construct the translation matrix from camera to robot
    cv::Mat trsl_C2R = (cv::Mat_<float>(3,1) << conf.x,
                                                conf.y,
                                                conf.z);

    // Construct the transition matrix from camera to robot
    cv::Mat Transi_C2R = (cv::Mat_<float>(4,4));
    cv::Mat Z = (cv::Mat_<float>(1,3) << 0,0,0);
    cv::Mat One = (cv::Mat_<float>(1,1) << 1);
    rot_C2R.copyTo(Transi_C2R(cv::Rect(0,0,rot_C2R.cols, rot_C2R.rows)));
    trsl_C2R.copyTo(Transi_C2R(cv::Rect(3,0,trsl_C2R.cols, trsl_C2R.rows)));
    Z.copyTo(Transi_C2R(cv::Rect(0,3,Z.cols, Z.rows)));
    One.copyTo(Transi_C2R(cv::Rect(3,3,One.cols, One.rows)));

    // Construct the transition matrix from robot to camera
    Transi_R2C = Transi_C2R.inv();
}

cv::Mat Cam::getMatI2C(){
    return K_I2C;
}

cv::Mat Cam::getMatC2I(){
    return K_C2I;
}

cv::Mat Cam::getMatC2R(){
    return Transi_C2R;
}

cv::Mat Cam::getMatR2C(){
    return Transi_R2C;
}

float Cam::getFocal(){
    return this->K_C2I.at<int>(0,0);
}
Vector2D<int> Cam::getSize(){
    Vector2D<int> size;
    size.x = (this->K_C2I.at<int>(0,2)*2) + 1;
    size.y = (this->K_C2I.at<int>(1,2)*2) + 1;

    return size;
}

ConfObj3D<float> Cam::getConf(){
    return conf;
}

float Cam::getAperAngle(){
    return aperAngl;
}
