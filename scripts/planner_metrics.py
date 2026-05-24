#!/usr/bin/env python3
import rclpy

from rclpy.node import Node
from nav_msgs.msg import Odometry

class PlannerMetrics(Node):
    def __init__(self):
        super().__init__('planner_metrics')

        self.odom_sub  = self.create_subscription(
            Odometry,
            'odom',
            self.odomCB,
            10
        )

        self.odom_sub
    
    def odomCB(self, msg):
        print("ODOM CB")


if __name__ == "__main__":
    rclpy.init(args=None)

    planner_metrics_node = PlannerMetrics()
    
    rclpy.spin(planner_metrics_node)