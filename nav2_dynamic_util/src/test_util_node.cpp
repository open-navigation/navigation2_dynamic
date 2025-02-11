#include "nav2_dynamic_util/tracked_obstacle_utils.hpp"
#include "nav2_dynamic_motion_model/constant_velocity_model.hpp" 


class TestObstacleNode : public rclcpp::Node
{
public:
    TestObstacleNode()
        : Node("test_obstacle_node")
    {
        motion_model_ = std::make_shared<nav2_dynamic_motion_model::ConstantVelocityModel>();
        testObstaclePrediction();
    }

private:
    std::shared_ptr<nav2_dynamic_motion_model::MotionModelInterface> motion_model_;

    void testObstaclePrediction()
    {
        nav2_dynamic_msgs::msg::Obstacle obstacle;
        obstacle.position.x = 1.0;
        obstacle.position.y = 2.0;
        obstacle.velocity.x = 0.5;
        obstacle.velocity.y = 0.2;
        double dt = 0.5;
        double sim_time = 4.0;

        for (; dt <= sim_time; dt += 0.5)
        {
            geometry_msgs::msg::Pose predicted_pose = nav2_dynamic_util::getObstaclePoseAt(dt, obstacle, motion_model_);
            RCLCPP_INFO(this->get_logger(), "Predicted Pose at t=%.1f: x=%.2f, y=%.2f", 
                        dt, predicted_pose.position.x, predicted_pose.position.y);
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TestObstacleNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
