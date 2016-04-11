#include "JointController.h"
#include <string>
//#include <sstream>
//#include "msgServosArm_hrrr.h"

int main(int argc, char** argv){
    JointController* jc = new JointController();

    // Arg1: Write pos values to motors, dflt = false
    jc->setPosControlEnabled(false);
    if(argc >= 2){
        std::string val(argv[1]);
        if(val == "true"  ||  val == "1")
            jc->setPosControlEnabled(true);
        else if(val != "false"  ||  val != "0")
            std::cout<<"arm_joint_controller: invalid first argument.\n";
    }
    else
        ROS_INFO("Input bool argument missing to write commands to the servos: default not write commands");

    // Setup ROS
    ros::init(argc, argv, "arm_joint_controller");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);  // Hz
    ROS_INFO("Controller node initialized");
    ROS_INFO("Namespace: %s", nh.getNamespace().c_str());

    // Initialize joint controller
    if(!jc->init(nh)){
        ROS_INFO("Failed to init joint controller");
        return -1;
    }

    // Start threaded spinner
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Main program loop
    ros::Time prevTime = ros::Time::now();
    while(ros::ok()){
        const ros::Time curTime = ros::Time::now();
        jc->_cm->update(curTime, curTime - prevTime);
        if(jc->getPosControlEnabled())
            jc->publishToArduino();
        jc->publishJointState();
        prevTime = curTime;

        loop_rate.sleep();
    }

    ros::waitForShutdown();
    return 0;
}
