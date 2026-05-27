#!/usr/bin/env python3
import pandas as pd
from pathlib import Path

import rclpy

from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from ament_index_python import get_package_prefix

class PlannerMetrics(Node):

    def __init__(self):
        super().__init__('planner_metrics')

        prefix = Path(get_package_prefix('ros2_dwa_pso'))
        prefix_ws = prefix.parent.parent

        self.csv_path = prefix_ws / 'src' / 'ros2_dwa_pso' / 'metrics'
        self.csv_path.mkdir(parents=True, exist_ok=True)

        self.odom_msg = Odometry()
        self.cmd_msg = Twist()

        self.path_data = pd.DataFrame(columns=['x','y'])
        self.time_data = pd.DataFrame(columns=['stamp', 'vel_trans', 'vel_ang'])

        self.cmd_count = 0
        self.odom_count = 0

        self.odom_sub  = self.create_subscription(
            Odometry,
            'odom',
            self.odomCB,
            10
        )
        self.cmd_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmdCB,
            10
        )

        self.update_odom_timer = self.create_timer(
            1.0,
            self.UpdateOdomCB
        )

        self.update_cmd_timer = self.create_timer(
            1.0,
            self.UpdateCmdCB
        )
    
    def odomCB(self, msg: Odometry):
        self.odom_msg = msg

    def cmdCB(self, msg: Twist):
        self.cmd_msg = msg

    def UpdateOdomCB(self):
        x = self.odom_msg.pose.pose.position.x
        y = self.odom_msg.pose.pose.position.y

        IS_POSE_NOT_ZERO = (x > 1e-3) or (y > 1e-3)

        if (self.path_data.shape[0] == 0) and IS_POSE_NOT_ZERO:
            self.path_data.loc[self.odom_count] = [x,y]
            print(f"\nWRITE ODOM: ({x},{y})")
            print(f"ODOM SIZE: {self.path_data.shape[0]}x{self.path_data.shape[1]}\n")
            self.odom_count += 1
        elif IS_POSE_NOT_ZERO:
            eps_x = abs(x - self.path_data.iloc[-1,0])
            eps_y = abs(y - self.path_data.iloc[-1,1])

            IS_NOT_STAT = (eps_x > 1e-3) or (eps_y > 1e-3)

            if IS_NOT_STAT:
                self.path_data.loc[self.odom_count] = [x,y]
                print(f"\nWRITE ODOM: ({x},{y})")
                print(f"ODOM SIZE: {self.path_data.shape[0]}x{self.path_data.shape[1]}\n")
                self.odom_count += 1

    def UpdateCmdCB(self):
        v = self.cmd_msg.linear.x
        omega = self.cmd_msg.angular.z

        stamp = self.get_clock().now().nanoseconds * 1e-9
        

        if v > 1e-3:
            self.time_data.loc[self.cmd_count] = [
                stamp,
                v,
                omega
            ]
            print(f"\nWRITE CMD: ({v},{omega})")
            print(f"CMD SIZE: {self.time_data.shape[0]}x{self.time_data.shape[1]}\n")
            self.cmd_count += 1

    def save_data(self):
        self.path_data.to_csv(self.csv_path / "path_data.csv",
                              encoding='utf-8',
                              index=False
        )
        self.time_data.to_csv(self.csv_path / "time_data.csv",
                              encoding='utf-8',
                              index=False
        )
        print("CSV files saved.")

if __name__ == "__main__":
    rclpy.init(args=None)

    planner_metrics_node = PlannerMetrics()
    
    try:
        rclpy.spin(planner_metrics_node)
    except KeyboardInterrupt:
        print("\nShutting down planner_metrics node ...")
    finally:
        planner_metrics_node.save_data()
        planner_metrics_node.destroy_node()
        rclpy.shutdown()