#ifndef DWA_PSO_PLANNER_HPP
#define DWA_PSO_PLANNER_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <mutex>
#include <atomic>

class DwaPsoPlanner : public rclcpp::Node {
    struct vel {
            double linear;
            double angular;
        };
    struct acc {
        double linear;
        double angular;
    };
    struct dynamic_limits {
        vel max_vel;
        acc max_acc;
    };

    public:
        DwaPsoPlanner();
    
    private:
        void odomCB(const nav_msgs::msg::Odometry::SharedPtr msg);
        void plannerCB();
        vel compute_dynamic_window(const nav_msgs::msg::Odometry odom);

        rclcpp::CallbackGroup::SharedPtr sub_group_;
        rclcpp::CallbackGroup::SharedPtr planner_group_;

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
        rclcpp::TimerBase::SharedPtr planner_timer_;

        std::mutex odom_mtx;
        nav_msgs::msg::Odometry last_odom;
        std::atomic<bool> have_odom{false};
        
        geometry_msgs::msg::Twist cmd_vel;

        // ROS-params                                                    
        dynamic_limits limits{{10.0, 5.0}, {2.0, 5.0}};
        double dt_ms{100.0};
};

#endif