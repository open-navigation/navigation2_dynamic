#include "nav2_dynamic_util/tracked_obstacle_utils.hpp"

namespace nav2_dynamic_util
{

    geometry_msgs::msg::Pose getObstaclePoseAt(const double dt, 
                                const nav2_dynamic_msgs::msg::Obstacle obstacle, 
                                const std::shared_ptr<nav2_dynamic_motion_model::MotionModelInterface> &motion_model)
    {
        
        geometry_msgs::msg::Pose predicted_pose = motion_model->predictObstaclePose(obstacle, dt);
        return predicted_pose;

    }
}
