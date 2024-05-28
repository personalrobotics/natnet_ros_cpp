#pragma once
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

namespace transforms {
    extern geometry_msgs::TransformStamped optitrackToBase;
    void initializeOptitrackToBase();
}