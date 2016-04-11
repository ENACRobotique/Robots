#ifndef JOINTCONTROLLER_H
#define JOINTCONTROLLER_H

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "arm_hrrr/msgServosArm_hrrr.h"
#include "controller_manager/controller_manager.h"
#include "ArmHrrrHW.h"

#define NB_MOTORS 4


class JointController{
public:
    JointController();
//    virtual ~JointController();

    bool init(ros::NodeHandle& nh);
    void publishJointState();
    void publishToArduino();
    void setPosControlEnabled(bool val);
    bool getPosControlEnabled() {return _posControlEnabled;}
    void setJntStateArdu(arm_hrrr::msgServosArm_hrrr& jntStateArdu, const sensor_msgs::JointState& jntState);
    void setJntState(sensor_msgs::JointState& jntState, const arm_hrrr::msgServosArm_hrrr& jntStateArdu);

    // Publish to arduino
    ros::Publisher _goal_jnt_state_ArduinoPub;

    // Publish to the joints state
    ros::Publisher _jntStatePub;

    // Subscribe to arduino (joints state)
    ros::Subscriber _jnt_state_ArduinoSub;
    void jntStateArduCb(const arm_hrrr::msgServosArm_hrrr& msg);

    // Subscribe to get set points
    ros::Subscriber _goal_jnt_state_Sub;
    void goalStateCb(const sensor_msgs::JointState& msg);

    //bool homeAllMotors(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    ArmHrrrHW* _armHrrrHw;
    controller_manager::ControllerManager* _cm;

private:
    bool _posControlEnabled;
    sensor_msgs::JointState _jnt_state;
    arm_hrrr::msgServosArm_hrrr _jnt_state_Arduino;
    arm_hrrr::msgServosArm_hrrr _goal_jnt_state_Arduino;
    ros::Time _timeLastGoalJntStatePub;
    int _goalJntStatePubPeriodInMSecs;
};

#endif // JOINTCONTROLLER_H
