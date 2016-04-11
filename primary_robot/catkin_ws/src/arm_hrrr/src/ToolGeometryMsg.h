#ifndef TOOLGEOMETRYMSG_H
#define TOOLGEOMETRYMSG_H

#include "ros/ros.h"

void copyGeometryMsg(const geometry_msgs::Pose ref, geometry_msgs::Pose& copy){
    copy.position.x = ref.position.x;
    copy.position.y = ref.position.y;
    copy.position.z = ref.position.z;

    copy.orientation.w = ref.orientation.w;
    copy.orientation.x = ref.orientation.x;
    copy.orientation.y = ref.orientation.y;
    copy.orientation.z = ref.orientation.z;
}

#endif // TOOLGEOMETRYMSG_H
