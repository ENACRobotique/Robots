#include "UI_functions.h"

//___________ Section to get information from the user
double getVal_UI(std::string name){
    std::string in;
    std::cout<<"Enter "<<name<<": ";
    std::cin>>in;
    return atof(in.c_str());
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

geometry_msgs::Pose setPoseFromUI(const bool useQuaternion, std::string rpy){
    geometry_msgs::Pose pose;
    pose.position.x = getVal_UI(std::string("x"));
    pose.position.y = getVal_UI(std::string("y"));
    pose.position.z = getVal_UI(std::string("z"));

    if(useQuaternion){
        double rollDeg;
        double pitchDeg;
        double yawDeg;
        if (rpy.find("r") != std::string::npos)
            rollDeg = getVal_UI(std::string("rollDeg"));
        else
            rollDeg = 0.;
        if (rpy.find("p") != std::string::npos)
            pitchDeg = getVal_UI(std::string("pitchDeg"));
        else
            rollDeg = 0.;
        if (rpy.find("y") != std::string::npos)
            yawDeg = getVal_UI(std::string("yawDeg"));
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

//___________ Section to print information
void printPose(std::string name, geometry_msgs::Pose& p){
    ROS_INFO("%s =(%.5f, %.5f, %.5f)", name.c_str(), p.position.x, p.position.y, p.position.z);
}

Eigen::Affine3d setAffine3d_UI(){
    return Eigen::Translation3d(getVal_UI("x"), getVal_UI("y"), getVal_UI("z"))*
           Eigen::Quaterniond(euler2Quaternion(getVal_UI("rollDeg"),
                                               getVal_UI("pitchDeg"),
                                               getVal_UI("yawDeg")));
}

Eigen::Affine3d setSuctCupAffine3d_UI(double x, double y, double z){
    double gamma = -atan2(x,y), beta = getVal_UI("coPitchDeg");
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
    box_pose.position.x = getVal_UI("x");
    box_pose.position.y = getVal_UI("y");
    box_pose.position.z = getVal_UI("z");
    Eigen::Quaterniond q = euler2Quaternion(getVal_UI("yawDeg"), 0., 0.);
    box_pose.orientation.x = q.x();
    box_pose.orientation.y = q.y();
    box_pose.orientation.z = q.z();
    box_pose.orientation.w = q.w();

    obj.primitives.push_back(primitive);
    obj.primitive_poses.push_back(box_pose);
    obj.operation = obj.ADD;

    return obj;
}

int getSelectedObj_UI(std::vector<moveit_msgs::CollisionObject>& objs){
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

eOrienTool getAttachDir_UI(){
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
