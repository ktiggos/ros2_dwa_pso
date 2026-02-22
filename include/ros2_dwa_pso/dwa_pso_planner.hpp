#ifndef DWA_PSO_PLANNER_HPP
#define DWA_PSO_PLANNER_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

class DwaPsoPlanner : public rclcpp::Node {
    public:
        DwaPsoPlanner();
    
    private:
        void odomCB(const nav_msgs::msg::Odometry::SharedPtr msg);

        rclcpp::CallbackGroup::SharedPtr sub_group_;
        rclcpp::CallbackGroup::SharedPtr planner_group_;

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;

        std::mutex odom_mtx;
        nav_msgs::msg::Odometry last_odom;
        bool have_odom {false};
};

#endif