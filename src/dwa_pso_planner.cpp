#include "ros2_dwa_pso/dwa_pso_planner.hpp"

DwaPsoPlanner::DwaPsoPlanner()
: Node("dwa_pso_planner")
{
    sub_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    
    rclcpp::SubscriptionOptions opts;
    opts.callback_group = sub_group_;

    // Reliable quality of service for odom sub
    rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

    sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom",
        qos,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg){
            this->odomCB(msg);
        },
        opts
    );

    opts.callback_group = planner_group_;

    planner_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        [this](){
            this->plannerCB();
        }
    );
}

void DwaPsoPlanner::plannerCB(){
    RCLCPP_INFO(this->get_logger(), "planner cb");
}

void DwaPsoPlanner::odomCB(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    std::lock_guard<std::mutex> lk(odom_mtx);
    last_odom = *msg;
    have_odom = true;
}

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);

    DwaPsoPlanner::SharedPtr node = std::make_shared<DwaPsoPlanner>();

    // Double threaded executor
    rclcpp::executors::MultiThreadedExecutor exec(
        rclcpp::ExecutorOptions(),
        2
    );

    exec.add_node(node);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}