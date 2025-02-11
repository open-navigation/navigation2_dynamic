#ifndef NAV2_DYNAMIC_UTIL__TRACKED_OBSTACLE_UTILS_HPP_
#define NAV2_DYNAMIC_UTIL__TRACKED_OBSTACLE_UTILS_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <nav2_dynamic_msgs/msg/obstacle.hpp>
#include <nav2_dynamic_msgs/msg/obstacle_array.hpp>
#include "nav2_dynamic_motion_model/motion_model_interface.hpp"

#include <random>
#include <vector>
#include <cmath>

namespace nav2_dynamic_util
{
    geometry_msgs::msg::Pose getObstaclePoseAt(
        double dt, 
        const nav2_dynamic_msgs::msg::Obstacle &obstacle, 
        const std::shared_ptr<nav2_dynamic_motion_model::MotionModelInterface> &motion_model);

    geometry_msgs::msg::PoseArray getObstaclePoseArrayAt(
        double dt, 
        const nav2_dynamic_msgs::msg::ObstacleArray &obstacle_array, 
        const std::shared_ptr<nav2_dynamic_motion_model::MotionModelInterface> &motion_model);

} // namespace
#endif  // NAV2_DYNAMIC_UTIL__TRACKED_OBSTACLE_UTILS_HPP_