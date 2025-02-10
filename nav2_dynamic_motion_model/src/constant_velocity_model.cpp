#include "nav2_dynamic_motion_model/constant_velocity_model.hpp"

namespace nav2_dynamic_motion_model
{

geometry_msgs::msg::Pose ConstantVelocityModel::predictObstaclePose(
        const nav2_dynamic_msgs::msg::Obstacle &obstacle,
        double dt) const
{
    geometry_msgs::msg::Pose predicted_pose;
    predicted_pose.position.x = obstacle.position.x;
    predicted_pose.position.y = obstacle.position.y;
    predicted_pose.position.z = obstacle.position.z;

    predicted_pose.position.x += obstacle.velocity.x * dt;
    predicted_pose.position.y += obstacle.velocity.y * dt;
    predicted_pose.position.z += obstacle.velocity.z * dt;
    return predicted_pose;
}

}

