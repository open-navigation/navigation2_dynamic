#include "nav2_dynamic_motion_model/constant_velocity_model.hpp"

namespace nav2_dynamic_motion_model
{

geometry_msgs::msg::Pose ConstantVelocityModel::predictObstaclePose(
    const nav2_dynamic_msgs::msg::Obstacle &obstacle,
    double dt) const
{
    geometry_msgs::msg::Pose predicted_pose;
    predicted_pose.position.x = obstacle.position.x + obstacle.velocity.x * dt;
    predicted_pose.position.y = obstacle.position.y + obstacle.velocity.y * dt;
    predicted_pose.position.z = obstacle.position.z + obstacle.velocity.z * dt;

    return predicted_pose;
}

geometry_msgs::msg::PoseArray ConstantVelocityModel::predictObstaclePoseArray(
    const nav2_dynamic_msgs::msg::ObstacleArray &obstacle_array,
    double dt) const
{
    geometry_msgs::msg::PoseArray pose_array;
    pose_array.poses.reserve(obstacle_array.obstacles.size());

    for (const auto &obstacle : obstacle_array.obstacles)
    {
        pose_array.poses.push_back(predictObstaclePose(obstacle, dt));
    }

    return pose_array;
}

} // namespace nav2_dynamic_motion_model
