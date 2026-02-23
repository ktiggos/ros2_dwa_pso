#include "ros2_dwa_pso/dwa_pso_planner.hpp"

DwaPsoPlanner::DwaPsoPlanner()
: Node("dwa_pso_planner") {
    sub_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    planner_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    
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

    planner_timer_ = this->create_wall_timer(
        std::chrono::milliseconds((int64_t)dt_ms),
        [this](){
            this->plannerCB();
        },
        planner_group_
    );
}

void DwaPsoPlanner::plannerCB() {
    nav_msgs::msg::Odometry odom;

    if(have_odom.load(std::memory_order_acquire)){
        std::lock_guard<std::mutex> lk(odom_mtx);
        odom = last_odom;
    }else{
        return;
    }

    RCLCPP_INFO(this->get_logger(),"DWA + PSO LOOP");
}

void DwaPsoPlanner::odomCB(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(odom_mtx);
    last_odom = *msg;
    have_odom.store(true, std::memory_order_release);
}

DwaPsoPlanner::vel DwaPsoPlanner::compute_dynamic_window(
    const nav_msgs::msg::Odometry odom
) {

    // Init dynamic window with vel limits
    vel wnd {
        this->limits.max_vel.linear,
        this->limits.max_vel.angular
    };

    const geometry_msgs::msg::Twist vel_prev {
        odom.twist.twist
    };

    double v_max {
        vel_prev.linear.x + limits.max_acc.linear * (dt_ms * 1e-3)
    };

    double w_max {
        vel_prev.angular.z + limits.max_acc.angular * (dt_ms * 1e-3)
    };

    // Update dynamic window based on acc limits
    wnd.linear = std::min(wnd.linear, v_max);
    wnd.angular = std::min(wnd.angular, w_max);

    return wnd;
}

int main(int argc, char* argv[]) {
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