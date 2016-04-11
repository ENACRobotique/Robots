#ifndef IK_ARM_HRRR_H
#define IK_ARM_HRRR_H

#include <Eigen/Dense>
#include <map>
#include <iostream>
#include <cmath>
#include <algorithm> // for min and max functions

class IK_arm_hrrr{
private:
    double l2_sq, l2;
    double l3_sq, l3;
    double l45_x, l45_z;
    double theta2_off;
    double theta1_off;
    double theta3_off;
    double x1_b, z1_b;
    Eigen::Vector3d pt_3;

    double thetamin[4], thetamax[4];

public:
    IK_arm_hrrr();
    bool setJntsFromIK(std::map<std::string, double>& joints,
             const double x, const double y, const double z, const double betaDeg);

};

#endif
