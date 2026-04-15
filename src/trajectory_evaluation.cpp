#include "ros2_dwa_pso/dwa_pso_planner.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/quaternion.hpp>

#include <random>
#include <vector>
#include <algorithm>
#include <limits>
#include <cmath>

static double wrap_angle(double a) {
    return std::atan2(std::sin(a), std::cos(a));
}

static double alpha_(const size_t k){
    return std::sqrt(k);
}

static double beta_(const uint q){
    if(q > 0){
        return 100.0;
    } else {
        return 0.0;
    }
}

double DwaPsoPlanner::eval_cost(const double v, const double w, const size_t k)
{
    double x_hat{0.0}, y_hat{0.0}, phi_hat{0.0};

    // Predict pose
    trajectory traj = eval_trajectory(odom, v, w);

    bool TRAJ_COLLISION = check_collision(traj);

    // if(TRAJ_COLLISION){
    //     RCLCPP_INFO(this->get_logger(), "COLLISION: %i", static_cast<int>(TRAJ_COLLISION));
    // }

    x_hat = traj.predicted_pose.x_hat;
    y_hat = traj.predicted_pose.y_hat;
    phi_hat = traj.predicted_pose.phi_hat;

    const double dx = this->goal.x - x_hat;
    const double dy = this->goal.y - y_hat;

    const double goal_bearing = std::atan2(dy, dx);
    const double head_score = std::cos(phi_hat - goal_bearing);

    const uint q = (TRAJ_COLLISION) ? 1 : 0;
    
    // Return objective function cost value
    return -(this->alpha * head_score + this->gamma * v
            - 100 * std::pow(beta_(q),2));
}

DwaPsoPlanner::trajectory DwaPsoPlanner::eval_trajectory(
    const nav_msgs::msg::Odometry& odom,
    const double v,
    const double w
) {
    constexpr double EPS_W = 1e-4;

    trajectory traj{};

    traj.vel.v = v;
    traj.vel.w = w;

    tf2::Quaternion q{
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
    // const double dt = this->dt_ms * 1e-3;
    const double dt = this->predict_time;

    double phi_hat = phi0 + w * dt;
    phi_hat = std::atan2(std::sin(phi_hat), std::cos(phi_hat));

    double x_hat, y_hat;

    if (std::fabs(w) > EPS_W) {
        const double s0 = std::sin(phi0);
        const double c0 = std::cos(phi0);
        const double s1 = std::sin(phi0 + w * dt);
        const double c1 = std::cos(phi0 + w * dt);

        const double R = v / w;

        x_hat = x0 + R * (s1 - s0);
        y_hat = y0 - R * (c1 - c0);

        traj.IS_LINEAR = false;
        traj.radius = std::abs(R);

        const double Mx = -R * std::sin(phi0);
        const double My = +R * std::cos(phi0);

        traj.center.xc = x0 + Mx;
        traj.center.yc = y0 + My;
    } else {
        x_hat = x0 + v * dt * std::cos(phi0);
        y_hat = y0 + v * dt * std::sin(phi0);

        traj.IS_LINEAR = true;
        traj.radius = 0.0;
        traj.center.xc = 0.0;
        traj.center.yc = 0.0;
    }

    traj.origin.x0 = x0;
    traj.origin.y0 = y0;
    traj.origin.phi0 = phi0;

    traj.predicted_pose.x_hat = x_hat;
    traj.predicted_pose.y_hat = y_hat;
    traj.predicted_pose.phi_hat = phi_hat;

    return traj;
}

bool DwaPsoPlanner::check_collision(trajectory t) {
    const double x0 = t.origin.x0;
    const double y0 = t.origin.y0;
    const double x_hat = t.predicted_pose.x_hat;
    const double y_hat = t.predicted_pose.y_hat;
    const double res = this->costmap.info.resolution;

    double x = x0;
    double y = y0;

    if (t.IS_LINEAR) {
        const double dx = x_hat - x0;
        const double dy = y_hat - y0;
        const double dist = std::hypot(dx, dy);

        const size_t pnum = static_cast<size_t>(std::ceil(dist / res));

        for (size_t i = 0; i <= pnum; i++) {
            const double s = std::min(static_cast<double>(i) * res, dist);
            const double u = (dist > 1e-9) ? s / dist : 0.0;
            x = x0 + u * dx;
            y = y0 + u * dy;

            const int c = get_cell_val(x, y);
            if (c < 0 || c >= this->thr_cost) {
                return true;
            }
        }
    } else {
        const double R = t.radius;
        const double xc = t.center.xc;
        const double yc = t.center.yc;

        const double theta0 = std::atan2(y0 - yc, x0 - xc);
        const double theta_hat = std::atan2(y_hat - yc, x_hat - xc);

        // Signed angular displacement following the motion direction
        double dtheta = wrap_angle(theta_hat - theta0);


        if (t.vel.w > 0.0) {
            if (dtheta < 0.0) {
                dtheta += 2.0 * M_PI;
            }
        } else {
            if (dtheta > 0.0) {
                dtheta -= 2.0 * M_PI;
            }
        }

        const double arc_len = std::abs(R * dtheta);
        const size_t pnum = static_cast<size_t>(std::ceil(arc_len / res));

        if (pnum == 0) {
            const int c0 = get_cell_val(x0, y0);
            if (c0 < 0 || c0 >= this->thr_cost) {
                return true;
            }

            const int c1 = get_cell_val(x_hat, y_hat);
            if (c1 < 0 || c1 >= this->thr_cost) {
                return true;
            }

            return false;
        }

        const double step_theta = dtheta / static_cast<double>(pnum);

        for (size_t i = 0; i <= pnum; i++) {
            const double theta = theta0 + static_cast<double>(i) * step_theta;
            x = xc + R * std::cos(theta);
            y = yc + R * std::sin(theta);

            const int c = get_cell_val(x, y);
            if (c < 0 || c >= this->thr_cost) {
                return true;
            }
        }
    }

    return false;
}

int DwaPsoPlanner::get_cell_val(double x, double y){
    const uint w = this->costmap.info.width;
    const uint h = this->costmap.info.height;
    const double lamda = this->costmap.info.resolution;
    const double x0 = this->costmap.info.origin.position.x;
    const double y0 = this->costmap.info.origin.position.y;

    int i = static_cast<int>(std::floor((x - x0) / lamda));
    int j = static_cast<int>(std::floor((y - y0) / lamda));

    if (i < 0 || j < 0 || i >= static_cast<int>(w) || j >= static_cast<int>(h)){
        return -1;
    }

    return this->costmap.data[j * w + i];
}