#include "IK_arm_hrrr.h"

#define RAD2DEG 180./M_PIl
#define DEG2RAD M_PIl/180.
#define SQ(x) (x*x)

IK_arm_hrrr::IK_arm_hrrr(){
    l2_sq = SQ(0.004) + SQ(0.102);
    l2 = sqrt(l2_sq);
    l3_sq = SQ(0.039) + SQ(0.08);
    l3 = sqrt(l3_sq);
    l45_x = 0.042 + 0.025;
    l45_z = 0.005;
    theta1_off = -atan2(-0.004, 0.102);
    theta2_off = atan2(0.035, 0.08);
    theta3_off = atan2(-0.005, 0.03 + 0.042);
    x1_b = 0.0365, z1_b = 0.1 + 0.03 + 0.0358;
    pt_3 << 0.072, 0, -0.005;

    thetamin[0] = -1.57;
    thetamax[0] = 1.57;
    thetamin[1] = -0.785;
    thetamax[1] = 2.356;
    thetamin[2] = -3.14;
    thetamax[2] = 0;
    thetamin[3] = -2.356;
    thetamax[3] = 0.785;
}

bool IK_arm_hrrr::setJntsFromIK(std::map<std::string, double>& joints,
         const double x, const double y, const double z, const double betaDeg){
    double beta = betaDeg*DEG2RAD;
    // Compute the value of "J0"
    joints["_J0"] = -atan2(x,y);

    // Work on the vertical plane xz and the frame of link1
    double r = sqrt(SQ(x) + SQ(y));
    double x1t_1 = r - x1_b, z1t_1 = z - z1_b;

    // Compute the origin O3 of the link 3 in the reference frame of link1

    Eigen::Vector3d pt_1, v;
    v << -l45_x*sin(beta) + l45_z*cos(beta) , 0.0, l45_x*cos(beta) + l45_z*sin(beta);
    pt_1 << x1t_1, 0, z1t_1;
    Eigen::Vector3d O3_1 = pt_1 + v;
    double xO3_1 = O3_1(0);
    double zO3_1 = O3_1(2);

    double r13_sq = SQ(xO3_1) + SQ(zO3_1);
    double nAlpha1 = (l3_sq - (l2_sq + r13_sq))/(-2*l2*sqrt(r13_sq));
    double arg1 = std::min(std::max(nAlpha1, -1.), 1.);
    double alpha1 = acos(arg1);
    double gamma1 = atan2(zO3_1, xO3_1);

    // Compute the value of "_J1"
    double theta1 = -M_PIl/2. + alpha1 + gamma1;
    joints["_J1"] = -theta1 + theta1_off;

    // Compute the value of "_J2"
    double nArg2 = (r13_sq - l2_sq - l3_sq)/(-2*l2*l3);
    double theta2 = acos(std::min(std::max(nArg2, -1.), 1.));
    joints["_J2"] = -theta2 + theta2_off;

    // Compute the value of "_J3"
    double theta3 = -theta1 + beta - theta2 + theta2_off;
    joints["_J3"] = -theta3 + theta3_off;

    std::cout<<"Joints value : [_JO, _J1, _J2, _J3] = "<<joints["_J0"]<<
               ", "<<joints["_J1"]<<", "<<joints["_J2"]<<", "<<joints["_J3"]<<
               "for [x, y,z, betaDeg] = "<<x<<", "<<y<<", "<<z<<", "<<betaDeg<<std::endl;

    // Check if the value are in the feasible domain
    bool isOutOfBound = false;
    if(joints["_J0"] < thetamin[0]  ||  joints["_J0"] > thetamax[0]){
        std::cout<<"Link "<<0<<" out of bounds: "<<thetamin[0]<<" <? "<<joints["_J0"]<<" <? "<<thetamax[0]<<std::endl;
        isOutOfBound = true;
    }
    if(joints["_J1"] < thetamin[1]  ||  joints["_J1"] > thetamax[1]){
        std::cout<<"Link "<<1<<" out of bounds: "<<thetamin[1]<<" <? "<<joints["_J1"]<<" <? "<<thetamax[1]<<std::endl;
        isOutOfBound = true;
    }
    if(joints["_J2"] < thetamin[2]  ||  joints["_J2"] > thetamax[2]){
        std::cout<<"Link "<<2<<" out of bounds: "<<thetamin[2]<<" <? "<<joints["_J2"]<<" <? "<<thetamax[2]<<std::endl;
        isOutOfBound = true;
    }
    if(joints["_J3"] < thetamin[3]  ||  joints["_J3"] > thetamax[3]){
        std::cout<<"Link "<<3<<" out of bounds: "<<thetamin[3]<<" <? "<<joints["_J3"]<<" <? "<<thetamax[3]<<std::endl;
        isOutOfBound = true;
    }
    if(isOutOfBound)
        return false;
    else
        return true;
}
