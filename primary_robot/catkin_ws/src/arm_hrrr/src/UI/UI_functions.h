#ifndef UI_FUNCTIONS_H
#define UI_FUNCTIONS_H

#include <string>
#include <geometry_msgs/Pose.h>
#include <moveit/move_group_interface/move_group.h>
#include <ros/console.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include "../tools/tools_functions.h"

//___________ Section to get information from the user
double getVal_UI(std::string name);
int getShape();
geometry_msgs::Pose setPoseFromUI(const bool useQuaternion = false, std::string rpy=std::string("rpy"));
Eigen::Affine3d setAffine3d_UI();
Eigen::Affine3d setSuctCupAffine3d_UI(double x, double y, double z);
moveit_msgs::CollisionObject addSandObj(const std::string id, const moveit::planning_interface::MoveGroup& group);
int getSelectedObj_UI(std::vector<moveit_msgs::CollisionObject>& objs);
eOrienTool getAttachDir_UI();


//___________ Section to print information
void printPose(std::string name, geometry_msgs::Pose& p);



//___________ Other
void addInitObjs(std::vector<moveit_msgs::CollisionObject>& coll_objs,
                 moveit::planning_interface::PlanningSceneInterface& planning_scene_interface,
                 moveit::planning_interface::MoveGroup& group);

#endif // UI_FUNCTIONS_H
