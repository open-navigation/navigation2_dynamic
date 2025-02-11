#ifndef NAV2_DYNAMIC_MOTION_MODEL__CONSTANT_VELOCITY_MODEL_HPP_
#define NAV2_DYNAMIC_MOTION_MODEL__CONSTANT_VELOCITY_MODEL_HPP_

#include "nav2_dynamic_motion_model/motion_model_interface.hpp"

namespace nav2_dynamic_motion_model
{

class ConstantVelocityModel : public MotionModelInterface
{
public:
    geometry_msgs::msg::Pose predictObstaclePose(
        const nav2_dynamic_msgs::msg::Obstacle &obstacle,
        double dt) const override;

    geometry_msgs::msg::PoseArray predictObstaclePoseArray(
        const nav2_dynamic_msgs::msg::ObstacleArray &obstacle_array,
        double dt) const override;
};

} // namespace nav2_dynamic_motion_model

#endif // NAV2_DYNAMIC_MOTION_MODEL__CONSTANT_VELOCITY_MODEL_HPP_
