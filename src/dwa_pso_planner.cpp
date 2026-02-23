#include "ros2_dwa_pso/dwa_pso_planner.hpp"

#include <tf2/tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/tf2/LinearMath/Quaternion.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

DwaPsoPlanner::DwaPsoPlanner()
: Node("dwa_pso_planner") {
    goal.x = 5.0;
    goal.y = 0.0;
    goal.z = 0.0;

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

    vel wnd = compute_dynamic_window(odom);

    RCLCPP_INFO(this->get_logger(),"%f, %f", wnd.linear, wnd.angular);
}

void DwaPsoPlanner::odomCB(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(odom_mtx);
    last_odom = *msg;
    have_odom.store(true, std::memory_order_release);
}

DwaPsoPlanner::vel DwaPsoPlanner::compute_dynamic_window(
    const nav_msgs::msg::Odometry& odom
) {
    const double dt{dt_ms * 1e-3};

    // Init dynamic window with vel limits
    vel wnd {
        this->limits.max_vel.linear,
        this->limits.max_vel.angular
    };

    const geometry_msgs::msg::Twist vel_curr {
        odom.twist.twist
    };

    double v_max = vel_curr.linear.x + limits.max_acc.linear * dt;
    double w_max = vel_curr.angular.z + limits.max_acc.angular * dt;

    // Update dynamic window based on acc limits
    wnd.linear = std::min(wnd.linear, v_max);
    wnd.angular = std::min(wnd.angular, w_max);

    return wnd;
}

double DwaPsoPlanner::eval_cost(const nav_msgs::msg::Odometry& odom,
                                const double v, const double w)
{
    double x_hat{0.0}, y_hat{0.0}, phi_hat{0.0};

    // Predict pose
    eval_trajectory(odom, v, w, x_hat, y_hat, phi_hat);

    const double dx = goal.x - x_hat;
    const double dy = goal.y - y_hat;

    const double goal_bearing = std::atan2(dy, dx);
    double head_score = std::cos(phi_hat - goal_bearing);

    // Return objective function cost value
    return -(this->alpha * head_score + this->gamma * v);
}

void DwaPsoPlanner::eval_trajectory(const nav_msgs::msg::Odometry& odom, 
    const double v, const double w,
    double &x_hat, double &y_hat, double &phi_hat
) {
    constexpr double EPS_W = 1e-4;

    tf2::Quaternion q {
        odom.pose.pose.orientation.x,
        odom.pose.pose.orientation.y,
        odom.pose.pose.orientation.z,
        odom.pose.pose.orientation.w
    };
    tf2::Matrix3x3 m{q};

    double roll, pitch, yaw; 
    m.getRPY(roll, pitch, yaw);

    const double x0 = odom.pose.pose.position.x;
    const double y0 = odom.pose.pose.position.y;
    const double phi0 = yaw;
    const double dt = dt_ms * 1e-3;

    // Predicted orientation
    phi_hat = phi0 + w * dt;
    phi_hat = std::atan2(std::sin(phi_hat), std::cos(phi_hat));

    // Predicted pose 
    if (std::fabs(w) > EPS_W) {
        // Constant-twist closed-form integration (circular arc)
        const double s0 = std::sin(phi0);
        const double c0 = std::cos(phi0);
        const double s1 = std::sin(phi0 + w * dt);
        const double c1 = std::cos(phi0 + w * dt);

        const double R = v / w;

        x_hat = x0 + R * (s1 - s0);
        y_hat = y0 - R * (c1 - c0);
    } else {
        // Straight-line approximation
        x_hat = x0 + v * dt * std::cos(phi0);
        y_hat = y0 + v * dt * std::sin(phi0);
    }
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