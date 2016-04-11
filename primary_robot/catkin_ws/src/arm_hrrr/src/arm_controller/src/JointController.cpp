#include "JointController.h"

JointController::JointController(){
    _timeLastGoalJntStatePub = ros::Time(0, 0);
    _goalJntStatePubPeriodInMSecs = 2000;

    _jnt_state.name.resize(NB_MOTORS);
    _jnt_state.position.resize(NB_MOTORS);
    _jnt_state.velocity.resize(NB_MOTORS);  // Not use
    _jnt_state.effort.resize(NB_MOTORS);  // Not use
}

bool JointController::init(ros::NodeHandle& nh){
    if(_posControlEnabled)
        ROS_WARN("Position controller enabled: commands will be sent to motors");
    else
        ROS_WARN("Position controller disabled: commands will not be sent to motors");

    // Init publishers
    _jntStatePub = nh.advertise<sensor_msgs::JointState>("arm_joint_states", 100);
    _goal_jnt_state_ArduinoPub = nh.advertise<arm_hrrr::msgServosArm_hrrr>("arm_goal_joint_states", 100);

    // Init subcribers
    _jnt_state_ArduinoSub = nh.subscribe("joint_state_Arduino", 100, &JointController::jntStateArduCb, this);
    _goal_jnt_state_Sub = nh.subscribe("/move_group/fake_controller_joint_states", 100, &JointController::goalStateCb, this);
    // Set the name of the joints
    _jnt_state.name[0] = "_J0";
    _jnt_state.name[1] = "_J1";
    _jnt_state.name[2] = "_J2";
    _jnt_state.name[3] = "_J3";

    // RobotHW interface for MoveIt!
    std::vector<std::string> jointNames(NB_MOTORS);
    for (int i=0; i< NB_MOTORS; ++i)
        jointNames[i] = _jnt_state.name[i];
    _armHrrrHw = new ArmHrrrHW(jointNames);
    _cm = new controller_manager::ControllerManager(_armHrrrHw);

    ROS_INFO("Joint controller initialized");
    return true;
}

/**
 * @brief JointController::write
 * @note Only position control
 */
void JointController::publishToArduino(){
    _goal_jnt_state_ArduinoPub.publish(_goal_jnt_state_Arduino);
}

void JointController::publishJointState(){
    _jntStatePub.publish(_jnt_state);
}

void JointController::setJntStateArdu(arm_hrrr::msgServosArm_hrrr& jntStateArdu, const sensor_msgs::JointState& jntState){
    for(int i=0; i<NB_MOTORS; i++){
        jntStateArdu.data[i] = static_cast<float>(jntState.position[i]);
    }
}

void JointController::setJntState(sensor_msgs::JointState& jntState, const arm_hrrr::msgServosArm_hrrr& jntStateArdu){
    for(int i=0; i<NB_MOTORS; i++){
        jntState.position[i] = static_cast<double>(jntStateArdu.data[i]);
    }
}

void JointController::jntStateArduCb(const arm_hrrr::msgServosArm_hrrr& msg){
    ROS_INFO("jointController: _jnt_state updated");
    setJntState(_jnt_state, msg);
}

void JointController::setPosControlEnabled(bool val){
    _posControlEnabled = val;
}

void JointController::goalStateCb(const sensor_msgs::JointState& msg){
    ROS_INFO("sensor_msgs::JointState received");
    for(int i=0; i<NB_MOTORS; i++){
        _goal_jnt_state_Arduino.data[i] = static_cast<float>(msg.position[i]);
    }
    ROS_INFO("_goal_jnt_state_Arduino updated");
}
