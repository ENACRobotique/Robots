#ifndef OBJSCONSTRUC_H
#define OBJSCONSTRUC_H

#include <string>
#include <utility>      // std::pair
#include <cstdint>

#include "ros/ros.h"
#include <moveit_msgs/CollisionObject.h>


#define DIMOBJ 0.058  // In mm


typedef enum eOrienTool{
    vertical,
    horizontal,
    eOrienToolMax
}eOrienTool;

typedef enum eActionManip{
    grabObj,
    putObj,
    eActionManipMax
}eActionManip;

typedef enum eTypeConstruc{
    tower11,
    tower21,
    wall2,
    wall3,
    eTypeConstrucMax
}eTypeConstruc;

typedef enum eShape{
    cub,
    cyl,
    con,
    eShapeMax
}eShape;

typedef std::tuple<eActionManip, eOrienTool, std::string, geometry_msgs::Pose> stepConstruc_t;
typedef std::vector<stepConstruc_t> planConstruc_t;
typedef std::vector<std::pair<bool, moveit_msgs::CollisionObject*>> objsConstruc_t;

class ObjsConstruc{
public:
    ObjsConstruc(eTypeConstruc type, geometry_msgs::Pose pose,
                 std::vector<moveit_msgs::CollisionObject*> objs);
    const planConstruc_t getPlanConstruc() const;
    void printStepConstruc(int i) const;
    void printPlanConstruc() const;

private:
    eTypeConstruc _type;
    geometry_msgs::Pose _pose;
    objsConstruc_t _objs;
    planConstruc_t _planConstruc;
    void makePlanConstruc();

    int findObj(uint8_t shape, bool isInConstruc);
};

std::string eActionManip2str(eActionManip act);
std::string eOrientationTool2str(eOrienTool orien);
std::string eTypeConstruc2str(eTypeConstruc type);
std::string eShape2str(eShape shape);


#endif // OBJSCONSTRUC_H
