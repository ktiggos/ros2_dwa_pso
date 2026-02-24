#include "ros2_dwa_pso/dwa_pso_planner.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/quaternion.hpp>

#include <random>
#include <vector>
#include <algorithm>
#include <limits>
#include <cmath>

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

    pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/cmd_vel",
        rclcpp::QoS(10)
    );

    planner_timer_ = this->create_wall_timer(
        std::chrono::milliseconds((int64_t)dt_ms),
        [this](){
            this->plannerCB();
        },
        planner_group_
    );
}

void DwaPsoPlanner::plannerCB()
{
    nav_msgs::msg::Odometry odom;

    if (this->have_odom.load(std::memory_order_acquire)) {
        std::lock_guard<std::mutex> lk(this->odom_mtx);
        odom = this->last_odom;
    } else {
        return;
    }

    const window wnd = this->compute_dynamic_window(odom);

    geometry_msgs::msg::Twist cmd_vel;

    // Init zero cmd before computation
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.linear.z = 0.0;

    cmd_vel.angular.x = 0.0;
    cmd_vel.angular.y = 0.0;
    cmd_vel.angular.z = 0.0;

    cmd_vel = this->pso_optimize_cmd(odom, wnd);

    // Optional safety: keep command inside window bounds
    cmd_vel.linear.x  = std::clamp(cmd_vel.linear.x,  wnd.v_min, wnd.v_max);
    cmd_vel.angular.z = std::clamp(cmd_vel.angular.z, wnd.w_min, wnd.w_max);

    pub_->publish(cmd_vel);
}

void DwaPsoPlanner::odomCB(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(odom_mtx);
    last_odom = *msg;
    have_odom.store(true, std::memory_order_release);
}

DwaPsoPlanner::window DwaPsoPlanner::compute_dynamic_window(const nav_msgs::msg::Odometry& odom)
{
    const double dt = this->dt_ms * 1e-3;

    const double v_curr = odom.twist.twist.linear.x;
    const double w_curr = odom.twist.twist.angular.z;

    const double dv = this->limits.max_acc.linear  * dt;
    const double dw = this->limits.max_acc.angular * dt;

    window wnd;
    wnd.v_min = std::clamp(v_curr - dv, -this->limits.max_vel.linear,  this->limits.max_vel.linear);
    wnd.v_max = std::clamp(v_curr + dv, -this->limits.max_vel.linear,  this->limits.max_vel.linear);

    wnd.w_min = std::clamp(w_curr - dw, -this->limits.max_vel.angular, this->limits.max_vel.angular);
    wnd.w_max = std::clamp(w_curr + dw, -this->limits.max_vel.angular, this->limits.max_vel.angular);

    return wnd;
}

geometry_msgs::msg::Twist DwaPsoPlanner::pso_optimize_cmd(
    const nav_msgs::msg::Odometry& odom,
    const window& wnd
)
{
    const double c1 = 2.0;
    const double c2 = 2.0;
    const double w_in  = 0.9;
    const double w_end = 0.4;

    const double v_min = wnd.v_min;
    const double v_max = wnd.v_max;
    const double w_min = wnd.w_min;
    const double w_max = wnd.w_max;

    if (v_max <= v_min || w_max <= w_min || this->n_par <= 0) {
        geometry_msgs::msg::Twist out;
        out.linear.x = 0.0;
        out.angular.z = 0.0;
        return out;
    }

    const double v_range = std::max(1e-9, v_max - v_min);
    const double w_range = std::max(1e-9, w_max - w_min);
    const double pv_max  = 0.5 * v_range;
    const double pw_max  = 0.5 * w_range;

    static thread_local std::mt19937 rng{std::mt19937::default_seed};
    std::uniform_real_distribution<double> uni01(0.0, 1.0);
    std::uniform_real_distribution<double> univ(v_min, v_max);
    std::uniform_real_distribution<double> uniw(w_min, w_max);

    struct Particle {
        double v{0.0}, w{0.0};
        double vv{0.0}, vw{0.0};
        double pbest_v{0.0}, pbest_w{0.0};
        double pbest_cost{std::numeric_limits<double>::infinity()};
        double cost{std::numeric_limits<double>::infinity()};
    };

    std::vector<Particle> swarm(static_cast<size_t>(this->n_par));

    for (auto &p : swarm) {
        p.v = univ(rng);
        p.w = uniw(rng);
        p.cost = this->eval_cost(odom, p.v, p.w);
        p.pbest_v = p.v;
        p.pbest_w = p.w;
        p.pbest_cost = p.cost;
    }

    double gbest_v = swarm.front().pbest_v;
    double gbest_w = swarm.front().pbest_w;
    double gbest_cost = swarm.front().pbest_cost;

    for (const auto &p : swarm) {
        if (p.pbest_cost < gbest_cost) {
            gbest_cost = p.pbest_cost;
            gbest_v = p.pbest_v;
            gbest_w = p.pbest_w;
        }
    }

    int stall = 0;
    double prev_gbest_cost = gbest_cost;

    for (size_t it = 0; it < this->imax; ++it) {

        const double tau = (this->imax > 1)
            ? static_cast<double>(it) / static_cast<double>(this->imax - 1)
            : 1.0;
        const double w_inertia = (1.0 - tau) * w_in + tau * w_end;

        for (auto &p : swarm) {

            const double r1 = uni01(rng);
            const double r2 = uni01(rng);
            const double r3 = uni01(rng);
            const double r4 = uni01(rng);

            p.vv = w_inertia * p.vv
                 + c1 * r1 * (p.pbest_v - p.v)
                 + c2 * r2 * (gbest_v   - p.v);

            p.vw = w_inertia * p.vw
                 + c1 * r3 * (p.pbest_w - p.w)
                 + c2 * r4 * (gbest_w   - p.w);

            p.vv = std::clamp(p.vv, -pv_max, pv_max);
            p.vw = std::clamp(p.vw, -pw_max, pw_max);

            p.v += p.vv;
            p.w += p.vw;

            p.v = std::clamp(p.v, v_min, v_max);
            p.w = std::clamp(p.w, w_min, w_max);

            p.cost = this->eval_cost(odom, p.v, p.w);

            if (p.cost < p.pbest_cost) {
                p.pbest_cost = p.cost;
                p.pbest_v = p.v;
                p.pbest_w = p.w;
            }
        }

        for (const auto &p : swarm) {
            if (p.pbest_cost < gbest_cost) {
                gbest_cost = p.pbest_cost;
                gbest_v = p.pbest_v;
                gbest_w = p.pbest_w;
            }
        }

        const double delta = std::fabs(prev_gbest_cost - gbest_cost);
        if (delta < this->eps_cost) ++stall;
        else stall = 0;

        prev_gbest_cost = gbest_cost;

        if (stall >= this->patience) break;
    }

    geometry_msgs::msg::Twist out;
    out.linear.x  = gbest_v;
    out.angular.z = gbest_w;
    return out;
}

double DwaPsoPlanner::eval_cost(const nav_msgs::msg::Odometry& odom,
                                const double v, const double w)
{
    double x_hat{0.0}, y_hat{0.0}, phi_hat{0.0};

    // Predict pose
    eval_trajectory(odom, v, w, x_hat, y_hat, phi_hat);

    const double dx = this->goal.x - x_hat;
    const double dy = this->goal.y - y_hat;

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
    const double dt = this->dt_ms * 1e-3;

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