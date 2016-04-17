//#include "moveit_api_test.h"
#include "ros/ros.h"
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
//#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <visualization_msgs/Marker.h>
#include <cstdint>

#include "arm.h"
#include "ObjsConstruc.h"
#include "UI/UI_functions.h"
#include "tools/tools_functions.h"
#include "tools/definition.h"

#include <stdlib.h>
#include <string>
#include <sstream>
#include "parameters.h"



int main(int argc, char **argv){
    double tSleep = 3.0;
    bool endProg = false, inOk = false;
    std::string sIn;
    int cpt = 0;

    ros::init(argc, argv, "moveit_api_test");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

#ifdef USE_UI
    sleep(7.0);  // To allow Rviz to come up
#endif

    moveit::planning_interface::MoveGroup groupArm0("arm0");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    geometry_msgs::Pose pose_eef = groupArm0.getCurrentPose().pose;
    printPose("pose_eef", pose_eef);

    const std::vector<std::string> vName = groupArm0.getActiveJoints();
    std::cout<<"Active joints:\n";
    for(int i=0;i<(int)vName.size(); i++)
        std::cout<<vName[i]<<std::endl;

    const moveit::core::JointModelGroup *jntModelGrp = groupArm0.getJointValueTarget().getJointModelGroup(groupArm0.getName());
    std::vector<std::string> jntNames = jntModelGrp->getVariableNames();
    for(int i=0; i<(int)jntNames.size(); i++)
        std::cout<<"Joint's name: "<<jntNames[i]<<std::endl;

    geometry_msgs::Pose start_pose = groupArm0.getCurrentPose().pose;
    geometry_msgs::Pose target_pose = groupArm0.getCurrentPose().pose;
    groupArm0.setPoseTarget(target_pose);
    printPose("start_pose", start_pose);
    printPose("target_pose", target_pose);

    ROS_INFO("Pose reference frame: %s", groupArm0.getPoseReferenceFrame().c_str());
    ROS_INFO("End-effector link: %s", groupArm0.getEndEffectorLink().c_str());
    ROS_INFO("End-effector: %s", groupArm0.getEndEffector().c_str());
    ROS_INFO("Planning frame: %s", groupArm0.getPlanningFrame().c_str());
//    groupArm0.setPlannerId("TRRTkConfigDefault");


    // Setting planning parameters
    groupArm0.setGoalTolerance(0.001);

    robot_state::RobotState curState = *groupArm0.getCurrentState();


    bool successPlan = false, successExec = false, targetPoseValid = false;
    std::string id_obj;
    robot_state::RobotState goalState = *groupArm0.getCurrentState();
    moveit::planning_interface::MoveGroup::Plan my_plan;
    std::map<std::string, double> joints;

    IK_arm_hrrr* Ik_arm = new IK_arm_hrrr();

    bool isPlanning = false;
    std::vector<moveit_msgs::CollisionObject> coll_objs;
    std::vector<std::string> objsToRemove;
    bool isObjattached = false;
    geometry_msgs::Pose centerConstruc;
    std::vector<ObjsConstruc*> listConstruc;
    std::vector<moveit_msgs::CollisionObject*> objsForConstruc;

    // Start mode loop
    while(!endProg){
        targetPoseValid = true;
        double x, y, z, beta;
        int selectedObj, attachedObj;
        std::ostringstream stm ;
        // Get the mode
        do{
            std::cout<<"Enter the desired mode:\n"
                        "\t q: Quit\n"
                        "\t 1: Move to restPose\n"
                        "\t 2: Move to maxRPose\n"
                        "\t 3: Print arm state\n"
                        "\t 4: Move to a specific pos/ori (x,y,z,beta) use personal IK\n"
                        "\t 5: Enter specific joint values\n"
                        "\t 6: Move to a random target\n"
                        "\t 7: Move to a specific orientation (rXDeg,rYDeg,rZDeg)\n"
                        "\t 8: Move suction cup to a specific pos/ori (x, y, z, beta)\n"
                        "\t 9: Print joints state\n"
                        "\t 10: Add init objects\n"
                        "\t 11: Add Sand object (with collisions checking)\n"
                        "\t 12: Move to object\n"
                        "\t 13: Attach object\n"
                        "\t 14: Detach object\n"
                        "\t 15: Removed an object\n"
                        "\t 16: Build a tower (1,1) (TODO)\n"
                        "\t 17: Build a tower (2,1) (TODO)\n";
            std::cin>>sIn;
            if(sIn == "q"  ||  (sIn >= "1"  &&  sIn <= "17"))
                inOk = true;
        }while(!inOk);

        // Update info about the arm
        curState = *groupArm0.getCurrentState();
        curState.printStatePositions();

        // Set the mode
        if(sIn == "q")
            endProg == true;
        else{
            int c = atoi(sIn.c_str());
            switch (c) {
            case 1:
                goalState.setToDefaultValues(curState.getJointModelGroup("arm0"), "restPose");
                if(!groupArm0.setJointValueTarget(goalState))
                    ROS_WARN("Wrong values");
                target_pose = groupArm0.getPoseTarget().pose;
                isPlanning = true;
                break;
            case 2:
                goalState.setToDefaultValues(curState.getJointModelGroup("arm0"), "maxRPose");
                if(!groupArm0.setJointValueTarget(goalState))
                    ROS_WARN("Wrong values");
                target_pose = groupArm0.getPoseTarget().pose;
                isPlanning = true;
                break;
            case 3:
                ROS_INFO("Print arm state\n");
                curState = *groupArm0.getCurrentState();
                curState.printStateInfo();
                isPlanning = false;
                break;
            case 4:
                ROS_INFO("Move to a specific pos/ori (x,y,z,beta) use personnal IK\n");
                Ik_arm->setJntsFromIK(joints, getVal_UI("x"), getVal_UI("y"), getVal_UI("z"), getVal_UI("beta_deg"));
                if(!groupArm0.setJointValueTarget(joints))
                    ROS_WARN("Wrong values");
                isPlanning = true;
                break;
            case 5:
                joints["_J0"]= getVal_UI("_J0");
                joints["_J1"]= getVal_UI("_J1");
                joints["_J2"]= getVal_UI("_J2");
                joints["_J3"]= getVal_UI("_J3");
                if(!groupArm0.setJointValueTarget(joints))
                    ROS_WARN("Wrong values");
                isPlanning = true;
                break;
            case 6:
                groupArm0.setRandomTarget();
                isPlanning = true;
                break;
            case 7:
                if(!groupArm0.setRPYTarget(getVal_UI("rollDeg")*DEG2RAD,
                                       getVal_UI("pitchDeg")*DEG2RAD,
                                       getVal_UI("yawDeg")*DEG2RAD))
                    ROS_WARN("Wrong values");
                isPlanning = true;
                break;
            case 8:
                ROS_INFO("Move suction cup to a specific pos/ori (x,y,z,beta)\n");
                x = getVal_UI("x"), y = getVal_UI("y"), z = getVal_UI("z");

                if(!goalState.setFromIK(goalState.getJointModelGroup("arm0"),
                                        setSuctCupAffine3d_UI(x, y, z))){
                    ROS_WARN("IK no solution found");
                    targetPoseValid = false;
                }
                if(!groupArm0.setJointValueTarget(goalState)  &&  targetPoseValid)
                    ROS_WARN("Wrong values");
                isPlanning = true;
                break;
            case 9:
                ROS_INFO("Print joints state\n");
                curState = *groupArm0.getCurrentState();
                std::cout<<"_J0 = "<<*curState.getJointPositions("_J0")<<std::endl;
                std::cout<<"_J1 = "<<*curState.getJointPositions("_J1")<<std::endl;
                std::cout<<"_J2 = "<<*curState.getJointPositions("_J2")<<std::endl;
                std::cout<<"_J3 = "<<*curState.getJointPositions("_J3")<<std::endl;
                isPlanning = false;
                break;
            case 10:
                ROS_INFO("Add init objects\n");
                addInitObjs(coll_objs, planning_scene_interface, groupArm0);
                sleep(1.0);
                break;
            case 11:
                ROS_INFO("Add sand object (with collisions checking)\n");
                stm << getVal_UI("id") ;
                id_obj = stm.str();
                std::cout<<"id_obj = "<<id_obj<<std::endl;
                coll_objs.push_back(addSandObj(id_obj, groupArm0));
                planning_scene_interface.addCollisionObjects(coll_objs);
                ROS_INFO("Add obj %s in into the world", id_obj.c_str());
                id_obj = "";
                sleep(1.0);
                groupArm0.setPlanningTime(5.0);
                break;
            case 12:
                ROS_INFO("Move to object\n");
                selectedObj = getSelectedObj_UI(coll_objs);
                if(selectedObj != -1){
                    // Get position of the object
                    x = coll_objs[selectedObj].primitive_poses[0].position.x;
                    y = coll_objs[selectedObj].primitive_poses[0].position.y;
                    z = coll_objs[selectedObj].primitive_poses[0].position.z;

                    // Compute position of the contact pt btw obj and arm
                    switch(getAttachDir_UI()){
                    case 0: // horizontal
                        if(x >= 0)
                            x -= DIMOBJ/2.*cos(atan2(x, y)) + COLL_MARGIN;
                        else
                            x += DIMOBJ/2.*cos(atan2(x, y)) + COLL_MARGIN;
                        y -= DIMOBJ*sin(atan2(x, y)) + COLL_MARGIN;
                        beta = 90.*DEG2RAD;
                        break;
                    default: // vertical
                        z += DIMOBJ/2 + COLL_MARGIN;
                        beta  = 0.0;
                    }

                    // Compute the goal position
                    Ik_arm->setJntsFromIK(joints, x, y, z, beta);
                    if(!groupArm0.setJointValueTarget(joints))
                        ROS_WARN("Wrong values");
                    isPlanning = true;
                }
                break;
            case 13:
                ROS_INFO("Attach object\n");
                if(!isObjattached){
                    attachedObj = selectedObj;
                    if(groupArm0.attachObject(coll_objs[selectedObj].id)){
                        isObjattached = true;
                        std::cout<<"isObjattached = "<<isObjattached<<std::endl;
                    }
                }
                else
                    ROS_WARN("Are you idiot! Detach the current object before to take another\n");
                break;
            case 14:
                ROS_INFO("Detach object\n");
                std::cout<<"isObjattached = "<<isObjattached<<std::endl;
                if(isObjattached){
                    if(groupArm0.detachObject(coll_objs[attachedObj].id)){
                        attachedObj = -1;
                        isObjattached = false;
                    }
                }
                else
                    ROS_WARN("Are you idiot! Attach an object first\n");
                break;
            case 15:
                ROS_INFO("Remove an object\n");
                selectedObj = getSelectedObj_UI(coll_objs);

                objsToRemove.push_back(coll_objs[selectedObj].id);
                planning_scene_interface.removeCollisionObjects(objsToRemove);
                objsToRemove.pop_back();
            case 16:
                ROS_INFO("Buld a tower (1,1)");
                if(isObjtsInWS(0, 1, 1, coll_objs)){
                    // Get the center of the construction
                    std::cout<<"Enter the center of the construction\n";
                    centerConstruc = setPoseFromUI(true, std::string("y"));
                    if(isConstruAreaFree(coll_objs, centerConstruc, tower11)){
                        objsForConstruc.push_back(findCollObj(coll_objs,
                                getSelectedIDObj(coll_objs, std::string("first object"))));
                        objsForConstruc.push_back(findCollObj(coll_objs,
                                getSelectedIDObj(coll_objs, std::string("second object"))));
                        ObjsConstruc* constr = new ObjsConstruc(tower11, centerConstruc, objsForConstruc);
                        listConstruc.push_back(constr);
                        makeConstruction(constr->getPlanConstruc(), coll_objs, groupArm0, true);
                    }
                }
                else{
                    ROS_WARN("Not enought objects to build the construction");
                }
                break;
            case 17:
                ROS_INFO("Buld a tower (2,1)");
                if(isObjtsInWS(0, 2, 1, coll_objs)){
                    // Get the center of the construction
                    std::cout<<"Enter the center of the construction\n";
                    centerConstruc = setPoseFromUI(true, std::string("y"));
                    if(isConstruAreaFree(coll_objs, centerConstruc, tower21)){
                        objsForConstruc.push_back(findCollObj(coll_objs,
                                getSelectedIDObj(coll_objs, std::string("first object"))));
                        objsForConstruc.push_back(findCollObj(coll_objs,
                                getSelectedIDObj(coll_objs, std::string("second object"))));
                        objsForConstruc.push_back(findCollObj(coll_objs,
                                getSelectedIDObj(coll_objs, std::string("third object"))));
                        ObjsConstruc* constr = new ObjsConstruc(tower21, centerConstruc, objsForConstruc);
                        listConstruc.push_back(constr);
                        makeConstruction(constr->getPlanConstruc(), coll_objs, groupArm0, true);
                    }
                }
                else{
                    ROS_WARN("Not enought objects to build the construction");
                }
                break;

            default:
                std::cout<<"Unknown mode "<<sIn<<std::endl;
                break;
            }
        }

        target_pose = groupArm0.getPoseTarget().pose;

        if(targetPoseValid  && isPlanning){
            // Plan and execute
            successPlan = groupArm0.plan(my_plan);
            printPose("target_pose", target_pose);
            ROS_INFO("Visualizing move %d = %s", cpt, successPlan ? "SUCCESS" : "FAILED");
            if(successPlan)
                successExec = groupArm0.execute(my_plan);
                ROS_INFO("Executing move %s", successExec ? "SUCCESS" : "FAILED");
            sleep(tSleep);  // To give Rviz time to visualize the plan
            cpt++;
        }
    }

    ROS_INFO("Exit");
    ros::shutdown();

    return 0;
}
