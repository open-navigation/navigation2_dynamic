#include "nav2_dynamic_util/tracked_obstacle_utils.hpp"

namespace nav2_dynamic_util
{

    geometry_msgs::msg::Pose getObstaclePoseAt(
        double dt, 
        const nav2_dynamic_msgs::msg::Obstacle &obstacle, 
        const std::shared_ptr<nav2_dynamic_motion_model::MotionModelInterface> &motion_model){
        return motion_model->predictObstaclePose(obstacle, dt);
    }

    geometry_msgs::msg::PoseArray getObstaclePoseArrayAt(
        double dt, 
        const nav2_dynamic_msgs::msg::ObstacleArray &obstacle_array, 
        const std::shared_ptr<nav2_dynamic_motion_model::MotionModelInterface> &motion_model){
        return motion_model->predictObstaclePoseArray(obstacle_array, dt);
    }

} // namespace nav2_dynamic_util
