#ifndef DWA_PSO_PLANNER_HPP
#define DWA_PSO_PLANNER_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <mutex>
#include <atomic>

class DwaPsoPlanner : public rclcpp::Node {

    public:
        DwaPsoPlanner();

        struct DynamicLimits {
            struct vel {
                double linear;
                double angular;
            };
            struct acc {
                double linear;
                double angular;
            };
            vel max_vel;
            acc max_acc;
        };

        struct window {
            double v_min, v_max;
            double w_min, w_max;
        };
    
    private:
        void odomCB(const nav_msgs::msg::Odometry::SharedPtr msg);

        void plannerCB();

        window compute_dynamic_window(const nav_msgs::msg::Odometry& odom);

        geometry_msgs::msg::Twist pso_optimize_cmd(
            const nav_msgs::msg::Odometry& odom, 
            const window& wnd
        );

        double eval_cost(const nav_msgs::msg::Odometry& odom, const double v, const double w);

        void eval_trajectory(const nav_msgs::msg::Odometry& odom,
            const double v, const double w,
            double &x_hat, double &y_hat, double &phi_hat
        );

        rclcpp::CallbackGroup::SharedPtr sub_group_;
        rclcpp::CallbackGroup::SharedPtr planner_group_;

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
        rclcpp::TimerBase::SharedPtr planner_timer_;

        std::mutex odom_mtx;
        nav_msgs::msg::Odometry last_odom;
        std::atomic<bool> have_odom{false};
        
        geometry_msgs::msg::Point goal;
        
        // ROS-params
        // DWA                                    
        double dt_ms{100.0};
        double eps_goal{1e-2};
        DynamicLimits limits{{10.0, 5.0}, {2.0, 5.0}};
        // PSO
        size_t imax{30}; // max iterations
        double eps_cost{1e-4};
        double eps_pos{1e-3};
        int patience{5};
        int n_par{30}; // particle number
        double alpha{1.0};
        double gamma{0.2}; 

};

#endif