#ifndef NAV2_DYNAMIC_MOTION_MODEL__MOTION_MODEL_INTERFACE_HPP_
#define NAV2_DYNAMIC_MOTION_MODEL__MOTION_MODEL_INTERFACE_HPP_

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav2_dynamic_msgs/msg/obstacle.hpp>
#include <nav2_dynamic_msgs/msg/obstacle_array.hpp>

namespace nav2_dynamic_motion_model
{

class MotionModelInterface
{
public:
    virtual ~MotionModelInterface() = default;

    virtual geometry_msgs::msg::Pose predictObstaclePose(
        const nav2_dynamic_msgs::msg::Obstacle &obstacle,
        double dt) const = 0;

    virtual geometry_msgs::msg::PoseArray predictObstaclePoseArray(
        const nav2_dynamic_msgs::msg::ObstacleArray &obstacle_array,
        double dt) const = 0;
};

} //namespace
#endif //NAV2_DYNAMIC_MOTION_MODEL__MOTION_MODEL_INTERFACE_HPP_