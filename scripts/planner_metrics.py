#!/usr/bin/env python3
import rclpy

from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class PlannerMetrics(Node):

    def __init__(self):
        super().__init__('planner_metrics')

        self.odom_msg = Odometry()
        self.cmd_msg = Twist()

        self.path = list()

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
        
    
    def __del__(self):
        print(self.path)
    
    def odomCB(self, msg: Odometry):
        self.odom_msg = msg

    def cmdCB(self, msg: Twist):
        self.cmd_msg = msg

    def UpdateOdomCB(self):
        x = self.odom_msg.pose.pose.position.x
        y = self.odom_msg.pose.pose.position.y

        IS_POSE_NOT_ZERO = (x > 1e-3) or (y > 1e-3)

        if (not self.path) and IS_POSE_NOT_ZERO:
            self.path.append((x,y))
        elif IS_POSE_NOT_ZERO:
            eps_x = abs(x - self.path[-1][0])
            eps_y = abs(y - self.path[-1][1])

            IS_NOT_STAT = (eps_x > 1e-3) and (eps_y > 1e-3)

            if IS_NOT_STAT:
                self.path.append(x,y)

if __name__ == "__main__":
    rclpy.init(args=None)

    planner_metrics_node = PlannerMetrics()
    
    rclpy.spin(planner_metrics_node)