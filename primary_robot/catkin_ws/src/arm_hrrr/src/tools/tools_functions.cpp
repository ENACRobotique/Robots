#include "tools_functions.h"


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

geometry_msgs::Pose set_gmPose(const double x, const double y, const double z,
                                  const double roll, const double pitch, const double yaw){
    geometry_msgs::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;

    Eigen::Quaterniond q = euler2Quaternion(roll, pitch, yaw);
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();

    return pose;
}


Eigen::Quaterniond euler2Quaternion(const double rollDeg, const double pitchDeg,
                                    const double yawDeg){
        Eigen::AngleAxisd rollAngle((rollDeg*DEG2RAD), Eigen::Vector3d::UnitZ());
        Eigen::AngleAxisd yawAngle((yawDeg*DEG2RAD), Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd pitchAngle((pitchDeg*DEG2RAD), Eigen::Vector3d::UnitX());

        Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
        return q;
}

///
/// \brief setAffine3d: Construct an Eigen::Affine3d object which represents a translation
///        and a quaternion
/// \param x
/// \param y
/// \param z
/// \param roll
/// \param pitch
/// \param yaw
/// \param conv2Rad is by default false and need to be true if the input angles are in degree.
/// \return an Eigen::Affine3d
///
Eigen::Affine3d setAffine3d(const double x, const double y, const double z,
                const double roll, const double pitch, const double yaw, const bool conv2Rad){
    if(conv2Rad){
        return Eigen::Translation3d(x, y, z)*
               Eigen::Quaterniond(euler2Quaternion(roll*DEG2RAD, pitch*DEG2RAD, yaw*DEG2RAD));
    }
    return Eigen::Translation3d(x, y, z)*
           Eigen::Quaterniond(euler2Quaternion(roll, pitch, yaw));
}

Eigen::Affine3d setEefAffine3d(const double x, const double y, const double z, const double beta,
                               const bool conv2Rad){
    double gamma = -atan2(x,y), betaRad = beta;

    if(conv2Rad)
        betaRad *= DEG2RAD;

    Eigen::Matrix3d R, Ru, Rsc;
    R = Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(0.0*M_PI_2, Eigen::Vector3d::UnitY())*
        Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ());

    Ru = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(gamma, Eigen::Vector3d::UnitZ())*
         Eigen::AngleAxisd(betaRad, Eigen::Vector3d::UnitX());

    Rsc = Ru*R;

    Eigen::Quaterniond quat = Eigen::Quaterniond(Rsc);
    return Eigen::Translation3d(x, y, z)*quat;
}

moveit_msgs::CollisionObject createSandObj(const std::string id, const moveit::planning_interface::MoveGroup& group,
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

bool isObjtsInWS(const eTypeConstruc type, std::vector<moveit_msgs::CollisionObject>& objs){
    int nbCub = 0, nbCyl = 0, nbCon = 0;
    switch (type) {
    case tower11:
        nbCyl = nbCon = 1;
        break;
    case tower21:
        nbCyl = 2;
        nbCon = 1;
        break;
    case wall222:
        nbCub = nbCyl = nbCon = 2;
        break;
    case wall322:
        nbCub = 3;
        nbCyl = nbCon = 2;
        break;
    default:
        ROS_WARN("Uknowm type of construction %d or not processed yet", type);
    }

    return isObjtsInWS(nbCub, nbCyl, nbCon, objs);
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
    return true;
}

bool planTo(moveit::planning_interface::MoveGroup& group, sArmPose goal, IK_arm_hrrr* Ik_arm){
    // Compute the goal position
    std::map<std::string, double> joints;
    bool successIK = Ik_arm->setJntsFromIK(joints, goal.x, goal.y, goal.z, goal.beta);
    if(!successIK){
        ROS_WARN("Planning IK FAILED");
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
        orienTool = horizontal;
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
        if(orienTool == vertical){
            ROS_WARN("Try again with horizontal orientation");
            orienTool = horizontal;
            if(!planToObj(collObj, group, orienTool)){
                ROS_WARN("Error again during planing");
                return false;
            }
        }else{
            return false;
        }
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
            moveit::planning_interface::MoveGroup& group, IK_arm_hrrr* Ik_arm){
    if(!planTo(group, goalArm, Ik_arm)){
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



// _________________________
bool makeConstruction(const planConstruc_t planConstruc, const std::vector<moveit_msgs::CollisionObject>& collObjs,
                      moveit::planning_interface::MoveGroup& group, IK_arm_hrrr* IK_arm, bool retToHome = false){
    int n = (int)planConstruc.size();
    std::cout<<"Start construction: "<<n<<" steps\n";
    moveit_msgs::CollisionObject* collObj;

    for(int i=0; i<n; i++){
        std::cout<<"Step "<<i<<std::endl;
        collObj = findCollObj(collObjs, std::get<2>(planConstruc[i]));
        if(!doActionManipObj(collObj, planConstruc[i], group, IK_arm)){
            return false;
        }
    }

    if(retToHome)
        if(!armRetToHome(group))
            return false;

    return true;
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

bool doActionManipObj(moveit_msgs::CollisionObject* collObj, stepConstruc_t stepConstruc,
                      moveit::planning_interface::MoveGroup& group, IK_arm_hrrr* Ik_arm){
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
        if(!armPutObj(collObj, armPose, group, Ik_arm)){
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
