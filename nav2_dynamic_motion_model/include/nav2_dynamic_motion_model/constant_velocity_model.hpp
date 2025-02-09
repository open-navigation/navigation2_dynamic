#ifndef NAV2_DYNAMIC_MOTION_MODEL__CONSTANT_VELOCITY_MODEL_HPP_
#define NAV2_DYNAMIC_MOTION_MODEL__CONSTANT_VELOCITY_MODEL_HPP_

#include "nav2_dynamic_motion_model/motion_model_base.hpp"


namespace nav2_dynamic_motion_model
{

class ConstantVelocityModel : public MotionModelInterface
{
public:
    geometry_msgs::msg::Pose predictObstaclePose(
        const nav2_dynamic_msgs::msg::Obstacle &obstacle,
        double dt) const override;
};


} //namespace
#endif //NAV2_DYNAMIC_MOTION_MODEL__CONSTANT_VELOCITY_MODEL_HPP_