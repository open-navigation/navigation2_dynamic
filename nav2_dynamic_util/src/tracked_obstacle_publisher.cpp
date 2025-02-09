#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "nav2_dynamic_msgs/msg/obstacle.hpp"  
#include "nav2_dynamic_msgs/msg/obstacle_array.hpp"  

#include <random>
#include <vector>
#include <cmath>

class ObstacleTracker : public rclcpp::Node {
public:
    ObstacleTracker() : Node("obstacle_tracker") {
        publisher_ = this->create_publisher<nav2_dynamic_msgs::msg::ObstacleArray>("/tracked_obstacles", 10);
        marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/obstacle_markers", 10);
        
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                         std::bind(&ObstacleTracker::update_obstacles, this));
        initialize_waypoints();
        initialize_obstacles();
    }

private:
    rclcpp::Publisher<nav2_dynamic_msgs::msg::ObstacleArray>::SharedPtr publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<nav2_dynamic_msgs::msg::Obstacle> obstacles_;
    std::vector<std::vector<geometry_msgs::msg::Point>> waypoints_;
    std::vector<size_t> waypoint_indices_;
    std::vector<double> progresses_;
    
    void initialize_waypoints() {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<float> pos_dist(-10.0, 10.0);
        
        waypoints_.resize(4);
        waypoint_indices_.resize(4, 0);
        progresses_.resize(4, 0.0);
        
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 5; ++j) {
                geometry_msgs::msg::Point p;
                p.x = pos_dist(gen);
                p.y = pos_dist(gen);
                p.z = 0.0;
                waypoints_[i].push_back(p);
            }
        }
    }

    void initialize_obstacles() {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<float> pos_dist(-10.0, 10.0);
        
        for (int i = 0; i < 4; ++i) {
            nav2_dynamic_msgs::msg::Obstacle obs;
            obs.uuid = generate_uuid();
            obs.score = 1.0;
            obs.position = waypoints_[i][0];
            obs.velocity.x = 0.0;
            obs.velocity.y = 0.0;
            obs.velocity.z = 0.0;
            obs.size.x = 1.0;
            obs.size.y = 1.0;
            obs.size.z = 1.0;
            std::fill(std::begin(obs.position_covariance), std::end(obs.position_covariance), 0.1);
            std::fill(std::begin(obs.velocity_covariance), std::end(obs.velocity_covariance), 0.05);
            
            obstacles_.push_back(obs);
        }
    }

    unique_identifier_msgs::msg::UUID generate_uuid() {
        unique_identifier_msgs::msg::UUID uuid;
        std::random_device rd;
        for (auto &byte : uuid.uuid) {
            byte = static_cast<uint8_t>(rd());
        }
        return uuid;
    }

    void update_obstacles() {
        if (obstacles_.empty() || waypoints_.empty()) return;
        
        nav2_dynamic_msgs::msg::ObstacleArray msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "map";
        
        visualization_msgs::msg::MarkerArray marker_array;
        int id = 0;

        for (size_t i = 0; i < obstacles_.size(); ++i) {
            geometry_msgs::msg::Point start = waypoints_[i][waypoint_indices_[i]];
            geometry_msgs::msg::Point end = waypoints_[i][(waypoint_indices_[i] + 1) % waypoints_[i].size()];
            
            progresses_[i] += 0.02; // Adjust speed of movement
            if (progresses_[i] >= 1.0) {
                progresses_[i] = 0.0;
                waypoint_indices_[i] = (waypoint_indices_[i] + 1) % waypoints_[i].size();
            }
            
            obstacles_[i].position.x = start.x + (end.x - start.x) * progresses_[i];
            obstacles_[i].position.y = start.y + (end.y - start.y) * progresses_[i];
            
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = this->now();
            marker.ns = "obstacles";
            marker.id = id++;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position = obstacles_[i].position;
            marker.scale.x = obstacles_[i].size.x;
            marker.scale.y = obstacles_[i].size.y;
            marker.scale.z = obstacles_[i].size.z;
            marker.color.a = 1.0;
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker_array.markers.push_back(marker);
        }
        
        msg.obstacles = obstacles_;
        publisher_->publish(msg);
        marker_publisher_->publish(marker_array);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObstacleTracker>());
    rclcpp::shutdown();
    return 0;
}
