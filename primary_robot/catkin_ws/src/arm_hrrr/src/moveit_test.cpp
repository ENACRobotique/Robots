#include "ros/ros.h"
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
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

#include "messages.h"
//#include <shared/botNet_core.h>
#include "/home/yoyo/Robots/static/communication/botNet/shared/botNet_core.h"
#include "/home/yoyo/Robots/static/communication/network_tools/bn_debug.h"
#include "/home/yoyo/Robots/static/communication/network_tools/bn_intp.h"


int main(int argc, char **argv){
    bool endProg = false;

    ros::init(argc, argv, "moveit_api_test");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

#ifdef USE_UI
    sleep(7.0);  // For init
#endif

    moveit::planning_interface::MoveGroup groupArm0("arm0");

    // Setting planning parameters
    groupArm0.setGoalTolerance(0.001);

    IK_arm_hrrr* Ik_arm = new IK_arm_hrrr();

    std::vector<moveit_msgs::CollisionObject> coll_objs;

    // Constructions
    geometry_msgs::Pose centerConstruc;
    std::vector<std::string> objsToRemove;
    std::vector<ObjsConstruc*> listConstruc;
    std::vector<moveit_msgs::CollisionObject*> objsForConstruc;

    sArmPose goal;
    eShape shape;
    sPickPutObject pickPutObj;
    sObject obj;
    sMakeConstruct infoConstruct;

    // Botnet
    sMsg inMsg = {{0}}, outMsg = {{0}};
    int ret;

    // botNet initialization
//    bn_attach(E_ROLE_SETUP, role_setup);
//    bn_attach(E_INTP, intp_sync_handler); // replaces bn_intp_install() to catch synchronization

    bn_init();

    // Start loop
    while(!endProg){
    	ret = bn_receive(&inMsg);

    	if(ret>0){
//            role_relay(&inMsg); // relay any received message if asked to

            switch(inMsg.header.type){
            case E_ARM_TARGET:
                inMsg.payload.goalTarget;
                goal.x = inMsg.payload.goalTarget.x;
                goal.y = inMsg.payload.goalTarget.y;
                goal.z = inMsg.payload.goalTarget.z;
                goal.beta = inMsg.payload.goalTarget.beta;
                ROS_INFO("Move arm to a specific pos/ori (x,y,z,beta) = %.2f, %.2f, %.2f, %.2f\n",
                         goal.x, goal.y, goal.z, goal.beta);

                planTo(groupArm0, goal, Ik_arm);
                moveToObj(groupArm0);
                break;
                // TODO
                break;
            case E_ARM_STATIC_CONF:
                inMsg.payload.staticConf.conf;
                armRetToHome(groupArm0); //TODO: Use several static pose
            	break;
            case E_ARM_PICK:
                pickPutObj = inMsg.payload.pickPutObj;
                shape = cub;
                goal.x = pickPutObj.x;
                goal.y = pickPutObj.y;
                goal.z = pickPutObj.z;
                goal.beta = pickPutObj.beta;
                coll_objs.push_back(createSandObj(std::to_string(pickPutObj.idObj), groupArm0, shape, goal)); // FIXME: Make a unique id
                armGrabObj(&(coll_objs.front()), groupArm0);
            	break;
            case E_ARM_PUT:
                pickPutObj = inMsg.payload.pickPutObj;
                goal.x = pickPutObj.x;
                goal.y = pickPutObj.y;
                goal.z = pickPutObj.z;
                goal.beta = pickPutObj.beta;
                armPutObj(findCollObj(coll_objs, "obj"), goal, groupArm0, Ik_arm); // FIXME: Make more flexible
                break;
            break;
            case E_ARM_CONSTRUCT:
                infoConstruct = inMsg.payload.makeConstruct;

                if(isObjtsInWS((eTypeConstruc) infoConstruct.type, coll_objs)){
                    // Get the center of the construction

                    centerConstruc = set_gmPose(infoConstruct.x, infoConstruct.y, infoConstruct.z,
                                                0., 0., infoConstruct.beta);
                    if(isConstruAreaFree(coll_objs, centerConstruc, (eTypeConstruc) infoConstruct.type)){
                        objsForConstruc.push_back(findCollObj(coll_objs,
                                getSelectedIDObj(coll_objs, std::string("first object")))); // FIXME
                        objsForConstruc.push_back(findCollObj(coll_objs,
                                getSelectedIDObj(coll_objs, std::string("second object"))));
                        ObjsConstruc* constr = new ObjsConstruc((eTypeConstruc) infoConstruct.type, centerConstruc, objsForConstruc); // FIXME
                        listConstruc.push_back(constr);
                        makeConstruction(constr->getPlanConstruc(), coll_objs, groupArm0, Ik_arm, true);
                    }
                }
                else{
                    ROS_WARN("Not enought objects to build the construction");
                }
			break;
            case E_ARM_OBJECT:
                obj = inMsg.payload.object;
                shape = cub;
                goal.x = obj.x;
                goal.y = obj.y;
                goal.z = obj.z;
                goal.beta = obj.beta;
                coll_objs.push_back(createSandObj(std::to_string(obj.idObj), groupArm0, shape, goal));
                ROS_INFO("Add new object (x,y,z,beta) = %.2f, %.2f, %.2f, %.2f\n",
                         goal.x, goal.y, goal.z, goal.beta);
                break;
            case E_DATA:
            case E_PING:
                break;
            default:
                bn_printfDbg("got unhandled msg: type%hhu sz%hhu", inMsg.header.type, inMsg.header.size);
                break;
            }
    	}


/*
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
*/

    }

    ROS_INFO("Exit");
    ros::shutdown();

    return 0;
}
