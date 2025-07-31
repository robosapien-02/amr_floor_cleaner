#!/usr/bin/env python3
import sys
import random
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class LaserFilter(Node):
    def __init__(self):
        super().__init__('LidarFilter')

        # Declare parameters (settable at launch or with ros2 param set)
        self.declare_parameter('start', True)
        self.declare_parameter('distance', 0.45)
        self.declare_parameter('Angle', 60)
        self.declare_parameter('linear', 0.8)
        self.declare_parameter('angular', 3.0)

        self.start = self.get_parameter('start').get_parameter_value().bool_value
        self.Dist = self.get_parameter('distance').get_parameter_value().double_value
        self.Angle = self.get_parameter('Angle').get_parameter_value().integer_value
        self.linear = self.get_parameter('linear').get_parameter_value().double_value
        self.angular = self.get_parameter('angular').get_parameter_value().double_value
        self.warning = [0,0,0]
        self.cmd = Twist()

        # Publisher and subscriber
        self.sub = self.create_subscription(LaserScan, "scan", self.callback, 10)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # Timer to drive the control loop at 10 Hz
        self.timer = self.create_timer(0.1, self.loop)

    def cancel(self):
        self.cmd_pub.publish(Twist())
        self.destroy_subscription(self.sub)
        self.destroy_publisher(self.cmd_pub)

    def callback(self, data):
        length = len(data.ranges)
        Index = int(self.Angle / 2 * length / 360)

        # middle 
        if (min(data.ranges[0:Index]) < self.Dist or min(data.ranges[(length - Index):]) < self.Dist):
            self.warning[1] = 1
        else:
            self.warning[1] = 0

        # left 
        left_start = max(int(length/4)-Index, 0)
        left_end = min(int(length/4)+Index, length)
        if min(data.ranges[left_start:left_end]) < self.Dist:
            self.warning[0] = 1
        else:
            self.warning[0] = 0

        # right 
        right_start = max(int(length*3/4)-Index, 0)
        right_end = min(int(length*3/4)+Index, length)
        if min(data.ranges[right_start:right_end]) < self.Dist:
            self.warning[2] = 1
        else:
            self.warning[2] = 0

    def loop(self):
        # Re-read parameters at every timer tick (Eloquent compatible)
        self.start = self.get_parameter('start').get_parameter_value().bool_value
        self.Dist = self.get_parameter('distance').get_parameter_value().double_value
        self.Angle = self.get_parameter('Angle').get_parameter_value().integer_value
        self.linear = self.get_parameter('linear').get_parameter_value().double_value
        self.angular = self.get_parameter('angular').get_parameter_value().double_value

        if self.start:
            if self.warning[1] == 0:    # forward
                self.cmd.linear.x = self.linear
                self.cmd.angular.z = 0.0
            elif self.warning[0] == 0:  # turn left
                self.cmd.linear.x = 0.0
                self.cmd.angular.z = self.angular
            elif self.warning[2] == 0:  # turn right
                self.cmd.linear.x = 0.0
                self.cmd.angular.z = -self.angular
            else:  # turn left or right randomly
                self.cmd.linear.x = 0.0
                self.cmd.angular.z = self.angular if random.randint(0,1) == 1 else -self.angular

            self.cmd_pub.publish(self.cmd)

def main(args=None):
    rclpy.init(args=args)
    node = LaserFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cancel()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

