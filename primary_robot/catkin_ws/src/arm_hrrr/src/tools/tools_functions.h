#ifndef TOOLS_FUNCTIONS_H
#define TOOLS_FUNCTIONS_H

#include <geometry_msgs/Pose.h>
#include "../arm.h"
#include <moveit/move_group_interface/move_group.h>
#include "../tools/definition.h"
#include "../objet.h"

#include "../ObjsConstruc.h" //FIXME: Not definef here
#include "../IK_arm_hrrr/IK_arm_hrrr.h" //FIXME: same

void setOrienInPose(geometry_msgs::Pose pos, eOrienTool orien);
Eigen::Quaterniond euler2Quaternion(const double rollDeg, const double pitchDeg,
                                    const double yawDeg);
Eigen::Affine3d setAffine3d(const double x, const double y, const double z,
                            const double roll, const double pitch, const double yaw, const bool conv2Rad = false);
Eigen::Affine3d setEefAffine3d(const double x, const double y, const double z, const double beta,
                               const bool conv2Rad = false);
moveit_msgs::CollisionObject createSandObj(const std::string id, const moveit::planning_interface::MoveGroup& group,
                                            const eShape shape, const sArmPose poseObj);
std::string getSelectedIDObj(std::vector<moveit_msgs::CollisionObject>& objs, std::string info);
bool isObjtsInWS(const int nbDesiCub, const int nbDesiCyl, const int nbDesiCon, std::vector<moveit_msgs::CollisionObject>& objs);
bool armRetToHome(moveit::planning_interface::MoveGroup& group);
bool moveToObj(moveit::planning_interface::MoveGroup& group);
bool planTo(moveit::planning_interface::MoveGroup& group, sArmPose goal, IK_arm_hrrr* Ik_arm);
bool setGoalArmWithObj(sArmPose& pose, const geometry_msgs::Pose poseObj, const eOrienTool orienTool);
bool planToObj(const moveit_msgs::CollisionObject* collObj,
               moveit::planning_interface::MoveGroup& group, eOrienTool orienTool);
bool armGrabObj(const moveit_msgs::CollisionObject* collObj,
              moveit::planning_interface::MoveGroup& group, eOrienTool orienTool = vertical);
bool armPutObj(moveit_msgs::CollisionObject* collObj, const sArmPose goalArm,
            moveit::planning_interface::MoveGroup& group, IK_arm_hrrr* Ik_arm);
moveit_msgs::CollisionObject* findCollObj(const std::vector<moveit_msgs::CollisionObject*> collObjs, std::string idObj);
moveit_msgs::CollisionObject* findCollObj(const std::vector<moveit_msgs::CollisionObject> collObjs, std::string idObj);

//_____ For construction
bool doActionManipObj(moveit_msgs::CollisionObject* collObj, stepConstruc_t stepConstruc,
                      moveit::planning_interface::MoveGroup& group, IK_arm_hrrr* Ik_arm);
bool isConstruAreaFree(std::vector<moveit_msgs::CollisionObject>& objs,
                       geometry_msgs::Pose center, eTypeConstruc type);
bool makeConstruction(const planConstruc_t planConstruc, const std::vector<moveit_msgs::CollisionObject>& collObjs,
                      moveit::planning_interface::MoveGroup& group, IK_arm_hrrr* IK_arm, bool retToHome);

#endif // TOOLS_FUNCTIONS_H
