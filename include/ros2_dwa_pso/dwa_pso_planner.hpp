#ifndef DWA_PSO_PLANNER_HPP
#define DWA_PSO_PLANNER_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

class DwaPsoPlanner : public rclcpp::Node {
    public:
        DwaPsoPlanner();
    
    private:
        rclcpp::Logger logger {this->get_logger()};
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
};

#endif