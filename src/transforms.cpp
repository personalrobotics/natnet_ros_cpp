#include "transforms.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

namespace transforms {
    geometry_msgs::TransformStamped optitrackToBase;
}

void transforms::initializeOptitrackToBase() {
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    transforms::optitrackToBase = tfBuffer.lookupTransform("base", "optitrack", ros::Time(0), ros::Duration(1.0));
}