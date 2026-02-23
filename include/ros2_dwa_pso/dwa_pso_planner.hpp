#ifndef DWA_PSO_PLANNER_HPP
#define DWA_PSO_PLANNER_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <mutex>
#include <atomic>

class DwaPsoPlanner : public rclcpp::Node {
    public:
        DwaPsoPlanner();
    
    private:
        void odomCB(const nav_msgs::msg::Odometry::SharedPtr msg);
        void plannerCB();

        rclcpp::CallbackGroup::SharedPtr sub_group_;
        rclcpp::CallbackGroup::SharedPtr planner_group_;

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
        rclcpp::TimerBase::SharedPtr planner_timer_;

        std::mutex odom_mtx;
        nav_msgs::msg::Odometry last_odom;
        std::atomic<bool> have_odom {false};
};

#endif