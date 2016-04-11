#ifndef ARMHRRRHW_H
#define ARMHRRRHW_H

#include <string>
#include <vector>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>


class ArmHrrrHW : public hardware_interface::RobotHW{
public:
    ArmHrrrHW(std::vector<std::string> jntNames);

    void setPos(const int& i, const double& val);
    double getPos(const int& i) const;
    void setCmd(const int& i, const double& val);
    double getCmd(const int& i) const;

private:
    int _N;
    std::vector<std::string> _name;
    std::vector<double> _pos;
    std::vector<double> _cmd;
    std::vector<double> _vel;
    std::vector<double> _eff;
    std::vector<hardware_interface::JointStateHandle*> _stateHandles;
    std::vector<hardware_interface::JointHandle*> _posHandles;
    hardware_interface::JointStateInterface _jointStateInterface;
    hardware_interface::PositionJointInterface _jointPosInterface;
};

#endif // ARMHRRRHW_H
