//#include "moveit_api_test.h"
#include "ros/ros.h"
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
//#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <visualization_msgs/Marker.h>
#include <cstdint>

#include "IK_arm_hrrr/IK_arm_hrrr.h"
#include "ObjsConstruc.h"

#include <stdlib.h>
#include <string>
#include <sstream>


#define DEG2RAD (M_PIl/180.)
#define RAD2DEG (180./M_PIl)
#define DIMOBJ 0.058  // In mm
#define COLL_MARGIN 0.005

#define SLEEP 3.0

typedef struct sArmPose{
    double x;
    double y;
    double z;
    double beta;
}sArmPose;


void printPose(std::string name, geometry_msgs::Pose& p){
    ROS_INFO("%s =(%.5f, %.5f, %.5f)", name.c_str(), p.position.x, p.position.y, p.position.z);
}

double getVal(std::string name){
    std::string in;
    std::cout<<"Enter "<<name<<": ";
    std::cin>>in;
    return atof(in.c_str());
}

void setOrienInPose(geometry_msgs::Pose pos, eOrienTool orien){
    switch(orien){
    case vertical:
        pos.orientation.x = -0.5;
        pos.orientation.y = 0.5;
        pos.orientation.z = 0.5;
        pos.orientation.w = 0.5;
        std::cout<<"Suction cup orientation: vertical\n";
        break;
    default:
        pos.orientation.x = 0.0;
        pos.orientation.y = 0.0;
        pos.orientation.z = 0.707106;
        pos.orientation.w = 0.707106;
        std::cout<<"Suction cup orientation: horizontal\n";
    }
}

Eigen::Quaterniond euler2Quaternion(const double rollDeg, const double pitchDeg,
                                    const double yawDeg){
        Eigen::AngleAxisd rollAngle((rollDeg*DEG2RAD), Eigen::Vector3d::UnitZ());
        Eigen::AngleAxisd yawAngle((yawDeg*DEG2RAD), Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd pitchAngle((pitchDeg*DEG2RAD), Eigen::Vector3d::UnitX());

        Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
        return q;
}

geometry_msgs::Pose setPose(const bool useQuaternion = false, std::string rpy=std::string("rpy")){
    geometry_msgs::Pose pose;
    pose.position.x = getVal(std::string("x"));
    pose.position.y = getVal(std::string("y"));
    pose.position.z = getVal(std::string("z"));

    if(useQuaternion){
        double rollDeg;
        double pitchDeg;
        double yawDeg;
        if (rpy.find("r") != std::string::npos)
            rollDeg = getVal(std::string("rollDeg"));
        else
            rollDeg = 0.;
        if (rpy.find("p") != std::string::npos)
            pitchDeg = getVal(std::string("pitchDeg"));
        else
            rollDeg = 0.;
        if (rpy.find("y") != std::string::npos)
            yawDeg = getVal(std::string("yawDeg"));
        else
            rollDeg = 0.;

        Eigen::Quaterniond q = euler2Quaternion(rollDeg, pitchDeg, yawDeg);
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        pose.orientation.w = q.w();
    }
    else{
        setOrienInPose(pose, vertical);
    }

    return pose;
}

Eigen::Affine3d setAffine3d(){
    return Eigen::Translation3d(getVal("x"), getVal("y"), getVal("z"))*
           Eigen::Quaterniond(euler2Quaternion(getVal("rollDeg"),
                                               getVal("pitchDeg"),
                                               getVal("yawDeg")));
}

Eigen::Affine3d setSuctCupAffine3d(double x, double y, double z){
    double gamma = -atan2(x,y), beta = getVal("coPitchDeg");
    Eigen::Matrix3d R, Ru, Rsc;
    R = Eigen::AngleAxisd(-M_PI/2.0, Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(0.0*M_PI/2.0, Eigen::Vector3d::UnitY())*
        Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitZ());

    Ru = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(gamma, Eigen::Vector3d::UnitZ())*
         Eigen::AngleAxisd(beta/180.0*M_PI, Eigen::Vector3d::UnitX());

    Rsc = Ru*R;
    std::cout<<"Rsc = \n"<<Rsc<<std::endl;

    Eigen::Quaterniond quat = Eigen::Quaterniond(Rsc);
    std::cout<<"beta = "<<beta<<", gamma = "<<gamma<<std::endl;
    std::cout<<"quaternion = "<<quat.x()<<", "<<quat.y()<<", "<<quat.z()<<", "<<quat.w()<<std::endl;
    return Eigen::Translation3d(x, y, z)*quat;
}
uint32_t shape = visualization_msgs::Marker::CUBE;

visualization_msgs::Marker addMarkerSand(const int id, const uint32_t type){
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/base_link";
    marker.header.stamp = ros::Time::now();
    marker.ns = "plgrd_obj";
    marker.id = id;
    marker.type = type;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = getVal("x");
    marker.pose.position.y = getVal("y");
    marker.pose.position.z = getVal("z");
    Eigen::Quaterniond q = euler2Quaternion(getVal("yawDeg"), 0., 0.);
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();
    marker.scale.x = DIMOBJ;
    marker.scale.y = DIMOBJ;
    marker.scale.z = DIMOBJ;
    marker.color.r = 232./255.;
    marker.color.g = 140./255.;
    marker.color.b = 0.;
    marker.color.a = 1.;
    marker.lifetime = ros::Duration();

    return marker;
}

int getShape(){
    std::string sIn;
    bool inOk = false;
    do{
        std::cout<<"Select the shape:\n"
                   "\t 0: cube (default)\n"
                   "\t 1: cylinder\n"
                   "\t 2: cone\n";
        std::cin>>sIn;
        if(sIn >= "0"  &&  sIn <= "2")
            inOk = true;
    }while(!inOk);

    return std::stoi(sIn);
}

moveit_msgs::CollisionObject addSandObj(const std::string id, const moveit::planning_interface::MoveGroup& group){
    moveit_msgs::CollisionObject obj;

    obj.header.frame_id = group.getPlanningFrame();
    obj.id = id;

    shape_msgs::SolidPrimitive primitive;
    switch(getShape()){
    case 1:
        primitive.type = primitive.CYLINDER;
        primitive.dimensions.resize(2);
        primitive.dimensions[0] = DIMOBJ;  // height
        primitive.dimensions[1] = DIMOBJ/2.;  // radius
        break;
    case 2:
        primitive.type = primitive.CONE;
        primitive.dimensions.resize(2);
        primitive.dimensions[0] = DIMOBJ;  // height
        primitive.dimensions[1] = DIMOBJ/2.;  // radius
        break;
    default:
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[0] = DIMOBJ;
        primitive.dimensions[1] = DIMOBJ;
        primitive.dimensions[2] = DIMOBJ;
    }


    geometry_msgs::Pose box_pose;
    box_pose.position.x = getVal("x");
    box_pose.position.y = getVal("y");
    box_pose.position.z = getVal("z");
    Eigen::Quaterniond q = euler2Quaternion(getVal("yawDeg"), 0., 0.);
    box_pose.orientation.x = q.x();
    box_pose.orientation.y = q.y();
    box_pose.orientation.z = q.z();
    box_pose.orientation.w = q.w();

    obj.primitives.push_back(primitive);
    obj.primitive_poses.push_back(box_pose);
    obj.operation = obj.ADD;

    return obj;
}

moveit_msgs::CollisionObject addInitSandObj(const std::string id, const moveit::planning_interface::MoveGroup& group,
                                            const eShape shape, const sArmPose poseObj){
    moveit_msgs::CollisionObject obj;

    obj.header.frame_id = group.getPlanningFrame();
    obj.id = id;

    shape_msgs::SolidPrimitive primitive;
    switch(shape){
    case cyl:
        primitive.type = primitive.CYLINDER;
        primitive.dimensions.resize(2);
        primitive.dimensions[0] = DIMOBJ;  // height
        primitive.dimensions[1] = DIMOBJ/2.;  // radius
        break;
    case con:
        primitive.type = primitive.CONE;
        primitive.dimensions.resize(2);
        primitive.dimensions[0] = DIMOBJ;  // height
        primitive.dimensions[1] = DIMOBJ/2.;  // radius
        break;
    default:
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[0] = DIMOBJ;
        primitive.dimensions[1] = DIMOBJ;
        primitive.dimensions[2] = DIMOBJ;
    }


    geometry_msgs::Pose box_pose;
    box_pose.position.x = poseObj.x;
    box_pose.position.y = poseObj.y;
    box_pose.position.z = poseObj.z;
    Eigen::Quaterniond q = euler2Quaternion(poseObj.beta, 0., 0.);
    box_pose.orientation.x = q.x();
    box_pose.orientation.y = q.y();
    box_pose.orientation.z = q.z();
    box_pose.orientation.w = q.w();

    obj.primitives.push_back(primitive);
    obj.primitive_poses.push_back(box_pose);
    obj.operation = obj.ADD;

    return obj;
}

int getSelectedObj(std::vector<moveit_msgs::CollisionObject>& objs){
    std::string sIn;
    std::ostringstream convert;
    convert <<(int)objs.size();
    bool inOk = false;
    do{ // Get the selected object
        std::cout<<"Select an object:\n";
        for(int i=0; i<(int)objs.size(); i++){
            std::cout<<"\t "<<i<<": id = "<<objs[i].id.c_str()<<std::endl;
        }
        std::cout<<"\t q: return\n";
        std::cin>>sIn;
        if(sIn == "q"  ||  (sIn >= "0"  &&  sIn < convert.str()))
            inOk = true;
    }while(!inOk);

    if(sIn == "q")
        return -1;

    return std::stoi(sIn);
}

std::string getSelectedIDObj(std::vector<moveit_msgs::CollisionObject>& objs, std::string info=std::string("")){
    std::string sIn;
    std::ostringstream convert;
    convert <<(int)objs.size();
    bool inOk = false;
    do{ // Get the selected object
        if(info.empty())
            std::cout<<"Select an object:\n";
        else
            std::cout<<"Select "<<info<<std::endl;
        for(int i=0; i<(int)objs.size(); i++){
            std::cout<<"\t "<<i<<": id = "<<objs[i].id.c_str()<<std::endl;
        }
        std::cout<<"\t q: return\n";
        std::cin>>sIn;
        if(sIn == "q"  ||  (sIn >= "0"  &&  sIn < convert.str()))
            inOk = true;
    }while(!inOk);

    if(sIn == "q")
        return std::string("");

    return objs[std::stoi(sIn)].id;
}

eOrienTool getAttachDir(){
    std::string sIn;
    bool inOk = false;
    do{ // Get the selected object
        std::cout<<"Enter the desired attach direction of the suction cup:\n"
                 "\t 0: horizontal (beta = 90°)\n"
                 "\t 1: vertical (default choice, beta = 0°)\n";
        std::cin>>sIn;
        if((sIn >= "0"  &&  sIn <= "1"))
            inOk = true;
    }while(!inOk);

    return (eOrienTool)std::stoi(sIn);
}

bool isObjtsInWS(const int nbDesiCub, const int nbDesiCyl, const int nbDesiCon, std::vector<moveit_msgs::CollisionObject>& objs){
    int nbObjs = (int)objs.size();
    int nbCub = 0, nbCon = 0, nbCyl = 0;
    uint8_t type;
    shape_msgs::SolidPrimitive prim;
    for(int i=0; i<nbObjs; i++){
        type = objs[i].primitives[0].type;
        if(type == prim.BOX){
            nbCub++;
            continue;
        }
        if(type == prim.CYLINDER){
            nbCyl++;
            continue;
        }
        if(type == prim.CONE){
            nbCon++;
        }
    }

    return ( (nbDesiCub <= nbCub)?(true):(false)  &&
             (nbDesiCyl <= nbCyl)?(true):(false)  &&
             (nbDesiCon <= nbCon)?(true):(false));
}

bool isConstruAreaFree(std::vector<moveit_msgs::CollisionObject>& objs,
                       geometry_msgs::Pose center, eTypeConstruc type){
    std::cout<<"isConstruAreaFree? ";
    int nbObjs = (int)objs.size();
    geometry_msgs::Pose posObj;
    double x_margin = DIMOBJ*3./2.;
    double y_margin = DIMOBJ*3./2.;

    switch(type){
    case(tower11):
        for(int i=0; i<nbObjs; i++){
            posObj = objs[i].primitive_poses[0];
            if(fabs(posObj.position.x - center.position.x) < x_margin)
                if(fabs(posObj.position.y - center.position.y) < y_margin){
                    std::cout<<"no\n";
                    ROS_WARN("Collision possible btw %s and the construction",
                             objs[i].id.c_str());
                    return false;
                }
        }
        std::cout<<"yes\n";
        return true;
    case(tower21):
        for(int i=0; i<nbObjs; i++){
            posObj = objs[i].primitive_poses[0];
            if(fabs(posObj.position.x - center.position.x) < x_margin)
                if(fabs(posObj.position.y - center.position.y) < y_margin){
                    std::cout<<"no\n";
                    ROS_WARN("Collision possible btw %s and the construction",
                             objs[i].id.c_str());
                    return false;
                }
        }
        std::cout<<"yes\n";
        return true;
    default:
        std::cout<<"no\n";
        ROS_WARN("isConstruAreaFree(): Type of construction %d unknown", type);
        return false;
    }
}

bool armRetToHome(moveit::planning_interface::MoveGroup& group){
    // Plan
    robot_state::RobotState goalState = *group.getCurrentState();
    goalState.setToDefaultValues(goalState.getJointModelGroup("arm0"), "restPose");
    if(!group.setJointValueTarget(goalState))
        ROS_WARN("Wrong values");

    // Move
    moveit::planning_interface::MoveGroup::Plan my_plan;
    bool successPlan = group.plan(my_plan), successExec = false;
    ROS_INFO("Planning move %s", successPlan ? "SUCCESS" : "FAILED");
    if(successPlan){
        successExec = group.execute(my_plan);
        ROS_INFO("Return to home %s", successExec ? "SUCCESS" : "FAILED");
        if(!successExec)
            return false;
    }
    else{
        return false;
    }
    sleep(SLEEP);  // To give Rviz time to visualize the plan
    return true;
}

bool moveToObj(moveit::planning_interface::MoveGroup& group){
    // Plan and execute
    moveit::planning_interface::MoveGroup::Plan my_plan;
    bool successPlan = group.plan(my_plan), successExec = false;
    ROS_INFO("Planning move %s", successPlan ? "SUCCESS" : "FAILED");
    if(successPlan){
        successExec = group.execute(my_plan);
        ROS_INFO("Executing move %s", successExec ? "SUCCESS" : "FAILED");
        if(!successExec)
            return false;
    }
    else{
        return false;
    }
    sleep(SLEEP);  // To give Rviz time to visualize the plan
    return true;
}

bool planTo(moveit::planning_interface::MoveGroup& group, sArmPose goal){
    // Compute the goal position
    IK_arm_hrrr* Ik_arm = new IK_arm_hrrr();
    std::map<std::string, double> joints;
    bool successIK = Ik_arm->setJntsFromIK(joints, goal.x, goal.y, goal.z, goal.beta);
    if(!successIK){
        ROS_INFO("Planning IK FAILED");
        return false;
    }
    if(!group.setJointValueTarget(joints)){
        ROS_WARN("Wrong values");
        return false;
    }
    return true;
}

bool setGoalArmWithObj(sArmPose& pose, const geometry_msgs::Pose poseObj, const eOrienTool orienTool){
    // Get position of the object
    double x = poseObj.position.x;
    double y = poseObj.position.y;
    double z = poseObj.position.z;
    std::cout<<"z = "<<z<<std::endl;

    // Compute position of the contact pt btw obj and arm
    switch(orienTool){
    case horizontal: // horizontal
        if(x >= 0)
            x -= DIMOBJ/2.*cos(atan2(x, y)) + COLL_MARGIN;
        else
            x += DIMOBJ/2.*cos(atan2(x, y)) + COLL_MARGIN;
        y -= DIMOBJ*sin(atan2(x, y)) + COLL_MARGIN;
        break;
    case vertical: // vertical
        z += DIMOBJ/2 + COLL_MARGIN;
        break;
    default:
        ROS_WARN("setGoalPoseObj(): Unknown orienTool: %d", orienTool);
        break;
    }

    pose.x = x;
    pose.y = y;
    pose.z = z;
    std::cout<<"pose.z = "<<pose.z<<std::endl;
    switch (orienTool) {
    case horizontal:
        pose.beta = 90.0*DEG2RAD;
        break;
    case vertical:
        pose.beta = 0.0;
        break;
    default:
        ROS_WARN("setGoalPoseObj(): Unknown orienTool: %d", orienTool);
        break;
    }
}

bool planToObj(const moveit_msgs::CollisionObject* collObj,
               moveit::planning_interface::MoveGroup& group, eOrienTool orienTool = eOrienToolMax){
    ROS_INFO("Move to object\n");
    bool successIK = false;

    // Get position of the object
    double x = collObj->primitive_poses[0].position.x;
    double y = collObj->primitive_poses[0].position.y;
    double z = collObj->primitive_poses[0].position.z;
    double beta;

    // Compute position of the contact pt btw obj and arm
    if(orienTool == eOrienToolMax)
        orienTool = getAttachDir();
    switch(orienTool){
    case horizontal: // horizontal
        if(x >= 0)
            x -= DIMOBJ/2.*cos(atan2(x, y)) + COLL_MARGIN;
        else
            x += DIMOBJ/2.*cos(atan2(x, y)) + COLL_MARGIN;
        y -= DIMOBJ*sin(atan2(x, y)) + COLL_MARGIN;
        beta = 90.*DEG2RAD;
        break;
    case vertical: // vertical
        z += DIMOBJ/2 + COLL_MARGIN;
        beta = 0.0;
    default:
        ROS_WARN("planToObj(): Unknown orienTool %d", orienTool);
    }

    // Compute the goal position
    IK_arm_hrrr* Ik_arm = new IK_arm_hrrr();
    std::map<std::string, double> joints;
    successIK = Ik_arm->setJntsFromIK(joints, x, y, z, beta);
    if(!successIK){
        ROS_INFO("Planning IK FAILED");
        return false;
    }
    if(!group.setJointValueTarget(joints)){
        ROS_WARN("Wrong values");
        return false;
    }
    return true;
}

bool armGrabObj(const moveit_msgs::CollisionObject* collObj,
              moveit::planning_interface::MoveGroup& group, eOrienTool orienTool){
    if(!planToObj(collObj, group, orienTool)){
        ROS_WARN("Error during planing");
        return false;
    }
    std::cout<<"armGrabObj(): Plan OK\n";
    if(!moveToObj(group)){
        ROS_WARN("Error during moving");
        return false;
    }
    std::cout<<"armGrabObj(): Move OK\n";
    if(!group.attachObject(collObj->id)){
        ROS_WARN("Error during attaching");
        return false;
    }
    std::cout<<"armGrabObj(): Attach OK\n";
    sleep(SLEEP);


    return true;
}

bool armPutObj(moveit_msgs::CollisionObject* collObj, const sArmPose goalArm,
            moveit::planning_interface::MoveGroup& group){
    if(!planTo(group, goalArm)){
        ROS_WARN("Error during planing");
        return false;
    }
    std::cout<<"armPutObj(): Plan OK\n";
    if(!moveToObj(group)){
        ROS_WARN("Error during moving");
        return false;
    }
    std::cout<<"armPutObj(): Move OK\n";

    collObj->primitive_poses[0].position.x = goalArm.x;  // FIXME not arm goal but obj goal
    collObj->primitive_poses[0].position.y = goalArm.y;  // FIXME not arm goal but obj goal
    collObj->primitive_poses[0].position.z = goalArm.z;  // FIXME not arm goal but obj goal

    if(!group.detachObject(collObj->id)){
        ROS_WARN("Error during attaching");
        return false;
    }
    std::cout<<"armPutObj(): Detach OK\n";

    return true;
}

bool doActionManipObj(moveit_msgs::CollisionObject* collObj, stepConstruc_t stepConstruc,
                      moveit::planning_interface::MoveGroup& group){
    sArmPose armPose;
    switch (std::get<0>(stepConstruc)) {
    case grabObj:
        if(!armGrabObj(collObj, group, std::get<1>(stepConstruc))){
            ROS_WARN("doActionManipObj(): Fail to grasp object %s", collObj->id.c_str());
            return false;
        }
        break;
    case putObj:
        setGoalArmWithObj(armPose, std::get<3>(stepConstruc), std::get<1>(stepConstruc) );
        if(!armPutObj(collObj, armPose, group)){
            ROS_WARN("doActionManipObj(): Fail to put object %s", collObj->id.c_str());
            return false;
        }
        break;
    default:
        ROS_WARN("doActionManipObj(): Unknown type of eActionManip: %d", std::get<0>(stepConstruc));
        return false;
    }
    return true;
}

moveit_msgs::CollisionObject* findCollObj(const std::vector<moveit_msgs::CollisionObject*> collObjs, std::string idObj){
    for(int i=0; i<(int)collObjs.size(); i++){
        if(collObjs[i]->id == idObj)
            return collObjs[i];
    }

    ROS_WARN("findCollObj(): Counld not find obj %s", idObj.c_str());
    return nullptr;
}

moveit_msgs::CollisionObject* findCollObj(const std::vector<moveit_msgs::CollisionObject> collObjs, std::string idObj){
    for(int i=0; i<(int)collObjs.size(); i++){
        if(collObjs[i].id == idObj)
            return new moveit_msgs::CollisionObject(collObjs[i]);
    }

    ROS_WARN("findCollObj(): Counld not find obj %s", idObj.c_str());
    return nullptr;
}

bool makeConstruction(const planConstruc_t planConstruc, const std::vector<moveit_msgs::CollisionObject>& collObjs,
                      moveit::planning_interface::MoveGroup& group, bool retToHome = false){
    int n = (int)planConstruc.size();
    std::cout<<"Start construction: "<<n<<" steps\n";
    moveit_msgs::CollisionObject* collObj;

    for(int i=0; i<n; i++){
        std::cout<<"Step "<<i<<std::endl;
        collObj = findCollObj(collObjs, std::get<2>(planConstruc[i]));
        if(!doActionManipObj(collObj, planConstruc[i], group)){
            return false;
        }
    }

    if(retToHome)
        if(!armRetToHome(group))
            return false;

    return true;
}

void addInitObjs(std::vector<moveit_msgs::CollisionObject>& coll_objs,
                 moveit::planning_interface::PlanningSceneInterface& planning_scene_interface,
                 moveit::planning_interface::MoveGroup& group){
    // Add cone
    coll_objs.push_back(addInitSandObj(std::string("0"), group, con, sArmPose{0.1, 0.1, 0.029, 0.}));
    std::cout<<"Add obj 0 into the world\n";

    // Add cylinder
    coll_objs.push_back(addInitSandObj(std::string("1"), group, cyl, sArmPose{0.15, 0.05, 0.029, 0.}));
    std::cout<<"Add obj 1 into the world\n";

    // Add cylinder
    coll_objs.push_back(addInitSandObj(std::string("2"), group, cyl, sArmPose{0.0, 0.1, 0.029, 0.}));
    std::cout<<"Add obj 2 into the world\n";

    planning_scene_interface.addCollisionObjects(coll_objs);
}


int main(int argc, char **argv){
    double tSleep = 3.0;
    bool endProg = false, inOk = false;
    std::string sIn;
    int cpt = 0;

    ros::init(argc, argv, "moveit_api_test");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    sleep(7.0);  // To allow Rviz to come up
    ROS_INFO("FINISH to sleep");

    moveit::planning_interface::MoveGroup groupArm0("arm0");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // To display markers
    ros::Publisher marker_pub = node_handle.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    // To visualize plans in Rviz
    ros::Publisher display_publisher =
            node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_groupArm0/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;


    std::cout<<"eef = "<<groupArm0.getEndEffector()<<std::endl;
//    groupArm0.setEndEffector(std::string("_tool"));
//    std::cout<<"eef = "<<groupArm0.getEndEffector()<<std::endl;
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
//    groupArm0.setPlannerId("SBLkConfigDefault");
//    groupArm0.setPlannerId("TRRTkConfigDefault");
//    groupArm0.setPlannerId("PRMkConfigDefault");

//    groupArm0.setEndEffector("endTool");
//    groupArm0.setEndEffectorLink("_tool");
//    ROS_INFO("End-effector link: %s", groupArm0.getEndEffectorLink().c_str());
//    ROS_INFO("End-effector: %s", groupArm0.getEndEffector().c_str());


    // Setting planning parameters
    groupArm0.setGoalTolerance(0.001);

    ROS_INFO("___ Current state ___:");
    robot_state::RobotState curState = *groupArm0.getCurrentState();
//    curState.printStateInfo();
//    curState.printStatePositions();
    ROS_INFO("_____________________:");

    bool successPlan = false, successExec = false, targetPoseValid = false;
    std::string id_obj;
    robot_state::RobotState goalState = *groupArm0.getCurrentState();
    moveit::planning_interface::MoveGroup::Plan my_plan;
    std::map<std::string, double> joints;

    IK_arm_hrrr* Ik_arm = new IK_arm_hrrr();

    bool isPlanning = false;
    visualization_msgs::Marker sandMarker;
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
                Ik_arm->setJntsFromIK(joints, getVal("x"), getVal("y"), getVal("z"), getVal("beta_deg"));
                if(!groupArm0.setJointValueTarget(joints))
                    ROS_WARN("Wrong values");
                isPlanning = true;
                break;
            case 5:
                joints["_J0"]= getVal("_J0");
                joints["_J1"]= getVal("_J1");
                joints["_J2"]= getVal("_J2");
                joints["_J3"]= getVal("_J3");
                if(!groupArm0.setJointValueTarget(joints))
                    ROS_WARN("Wrong values");
                isPlanning = true;
                break;
            case 6:
                groupArm0.setRandomTarget();
                isPlanning = true;
                break;
            case 7:
                if(!groupArm0.setRPYTarget(getVal("rollDeg")*DEG2RAD,
                                       getVal("pitchDeg")*DEG2RAD,
                                       getVal("yawDeg")*DEG2RAD))
                    ROS_WARN("Wrong values");
                isPlanning = true;
                break;
            case 8:
                ROS_INFO("Move suction cup to a specific pos/ori (x,y,z,beta)\n");
                x = getVal("x"), y = getVal("y"), z = getVal("z");

                if(!goalState.setFromIK(goalState.getJointModelGroup("arm0"),
                                        setSuctCupAffine3d(x, y, z))){
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
                stm << getVal("id") ;
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
                selectedObj = getSelectedObj(coll_objs);
                if(selectedObj != -1){
                    // Get position of the object
                    x = coll_objs[selectedObj].primitive_poses[0].position.x;
                    y = coll_objs[selectedObj].primitive_poses[0].position.y;
                    z = coll_objs[selectedObj].primitive_poses[0].position.z;

                    // Compute position of the contact pt btw obj and arm
                    switch(getAttachDir()){
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
                selectedObj = getSelectedObj(coll_objs);

                objsToRemove.push_back(coll_objs[selectedObj].id);
                planning_scene_interface.removeCollisionObjects(objsToRemove);
                objsToRemove.pop_back();
            case 16:
                ROS_INFO("Buld a tower (1,1)");
                if(isObjtsInWS(0, 1, 1, coll_objs)){
                    // Get the center of the construction
                    std::cout<<"Enter the center of the construction\n";
                    centerConstruc = setPose(true, std::string("y"));
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
                    centerConstruc = setPose(true, std::string("y"));
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



    // Planning to a Pose goal
//    std::cout<<"\n___ First move ___\n";
//    target_pose = groupArm0.getCurrentPose().pose;
//    target_pose.position.x += 0.00;    // Foot forward
//    target_pose.position.y += 0.1;    // Foot out
//    target_pose.position.z -= 0.05;      // Foot up
//    printPose("target_pose1", target_pose);
//    groupArm0.setJointValueTarget(target_pose);  // This also works!

//    robot_state::RobotState goal_state = *groupArm0.getCurrentState();
//    cur_state.setToDefaultValues(cur_state.getJointModelGroup("arm0"), "restPose");
//    cur_state.setToDefaultValues(goal_state.getJointModelGroup("arm0"), "maxRPose");
//    groupArm0.setStartState(goal_state);
//    groupArm0.setJointValueTarget(cur_state);
//    target_pose = groupArm0.getPoseTarget().pose;
//    printPose("target_pose3", target_pose);

    // Call the planner to compute the plan and visualize it.
    // Note: Just planning, not asking move_group to actually move the robot.
//    moveit::planning_interface::MoveGroup::Plan my_plan;
//    successPlan = groupArm0.plan(my_plan);
//    ROS_INFO("Visualizing move %s", successPlan ? "SUCCESS" : "FAILED");
//    if(successPlan)
//        successExec = groupArm0.execute(my_plan);
//        ROS_INFO("Executing move %s", successExec ? "SUCCESS" : "FAILED");

//    sleep(tSleep);  // To give Rviz time to visualize the plan


    // Second try
//    std::cout<<"\n___ Second move ___\n";
//    target_pose = groupArm0.getCurrentPose().pose;
//    target_pose.position.x += 1.00;    // Foot forward
//    target_pose.position.y += 1.5;    // Foot out
//    target_pose.position.z -= 1.05;      // Foot up
//    printPose("target_pose2", target_pose);
//    groupArm0.setPoseTarget(target_pose);
//    groupArm0.setJointValueTarget(target_pose);  // This also works!

//    successPlan = groupArm0.plan(my_plan);
//    ROS_INFO("Visualizing move %s", successPlan ? "SUCCESS" : "FAILED");
//    if(successPlan)
//        successExec = groupArm0.execute(my_plan);
//        ROS_INFO("Executing move %s", successExec ? "SUCCESS" : "FAILED");
//    sleep(tSleep);  // To give Rviz time to visualize the plan



    // Third try
//    std::cout<<"\n___ Third move ___\n";
//    groupArm0.setRandomTarget();
//    target_pose = groupArm0.getPoseTarget().pose;
////    printPose("target_pose3", target_pose);
//    groupArm0.move();
//    sleep(tSleep);

    // Fourth try
//    std::cout<<"\n___ Fourth move ___\n";
//    groupArm0.setNamedTarget("straightUp");
////    std::map<std::string, double> joints;
////    joints["_J0"]= 0.0;
////    joints["_J1"]= 1.0;
////    joints["_J2"]= 1.0;
////    joints["_J3"]= 2.0;
////    groupArm0.setJointValueTarget(joints);
////    target_pose = groupArm0.getPoseTarget().pose;
//    printPose("target_pose4", target_pose);
//    groupArm0.move();
//    sleep(tSleep);

    // Fifth try
//    std::cout<<"\n___ Fifth move ___\n";
//    Eigen::Affine3d pose = Eigen::Translation3d(0.1, 0.2, 0.1)
//            *Eigen::Quaterniond(0.69, 0.108, 0.706, -0.112);
//    groupArm0.setPoseTarget(pose);
//    target_pose = groupArm0.getPoseTarget().pose;
//    printPose("target_pose5", target_pose);
//    groupArm0.move();
//    sleep(tSleep);

    // Manual target
//    bool go = true;
//    std::string s;
//    int cpt = 0;
//    do{
//        std::cout<<"\n___ Manual move ___\n";
//        target_pose = setPose();
//        groupArm0.setJointValueTarget(target_pose);
//        printPose("target_pose", target_pose);
//        successPlan = groupArm0.plan(my_plan);
//        ROS_INFO("Visualizing move %d = %s", cpt, successPlan ? "SUCCESS" : "FAILED");
//        if(successPlan)
//            successExec = groupArm0.execute(my_plan);
//            ROS_INFO("Executing move %s", successExec ? "SUCCESS" : "FAILED");
//        sleep(tSleep);  // To give Rviz time to visualize the plan
//        cpt++;
//        std::cout<<"Continue? press 'q'\n";
//        std::cin>>s;
//        if(s=="q")
//             go = false;
//    }while(go);



    //______ Cartesian Paths ______
//    std::vector<geometry_msgs::Pose> waypoints;

//    target_pose.position.x += 0.05;
//    target_pose.position.z -= 0.02;
//    waypoints.push_back(target_pose);  // up and out
//
//    target_pose.position.y -= 0.02;
//    waypoints.push_back(target_pose);  // right
//
//    target_pose.position.z += 0.02;
//    target_pose.position.y += 0.02;
//    target_pose.position.x -= 0.02;
//    waypoints.push_back(target_pose);  // back to start

//    target_pose.orientation.w = sqrt(2.0);
//    target_pose.orientation.x = 0.0;
//    target_pose.orientation.y = 0.0;
//    target_pose.orientation.z = sqrt(2.0);
//    waypoints.push_back(target_pose);


//    // Ellipse
//    start_pose = groupArm0.getCurrentPose().pose;
//    target_pose = start_pose;
//    float ellipseA = 0.02;
//    float ellipseB = 0.02;
//    int N = 50;
//    for (int i = 0; i < N; ++i)
//    {
//        float t = i/(float)(N)*2*M_PI;
//        target_pose.position.x = ellipseA*cos(t) + start_pose.position.x;
//        target_pose.position.z = ellipseB*sin(t) + start_pose.position.z + ellipseB;
//        waypoints.push_back(target_pose);
//        ROS_INFO("Pose x: %g, z: %g", target_pose.position.x, target_pose.position.z);
//    }

//    //target_pose.position.x += 0.02;    // Foot forward
//    //target_pose.position.y -= 0.02;    // Foot out
//    target_pose.position.z += 0.02;      // Foot up
//    waypoints.push_back(target_pose);

//    groupArm0.setGoalTolerance(0.01);

//    moveit_msgs::RobotTrajectory trajectory;
//    double fraction;
//    fraction = groupArm0.computeCartesianPath(waypoints, 0.01, 0.0, trajectory, false);
//    ROS_INFO("Visualizing cartesian path (%.2f%% achieved)", fraction * 100.0);
//    // Sleep to give Rviz time to visualize the plan
//    sleep(10.0);


    ROS_INFO("Exit");
    ros::shutdown();

    return 0;
}
