#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, Quaternion
from nav_msgs.msg import Odometry
from math import radians, degrees, copysign, sqrt, pow, pi
from rclpy.parameter import Parameter
from tf_transformations import euler_from_quaternion
from tf2_ros import TransformListener, Buffer
import time

class CalibrateMotion(Node):
    def __init__(self):
        super().__init__('calibrate_motion')
        # Mode: "linear" or "angular"
        self.declare_parameter('mode', 'linear')
        self.mode = self.get_parameter('mode').get_parameter_value().string_value

        # Shared
        self.declare_parameter('rate', 20)
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('odom_frame', 'odom')
        self.cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.main_loop)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.start_test = True

        if self.mode == 'linear':
            # Linear calibration params
            self.declare_parameter('test_distance', 1.0)
            self.declare_parameter('speed', 0.3)
            self.declare_parameter('tolerance', 0.01)
            self.declare_parameter('odom_linear_scale_correction', 1.0)
        else:
            # Angular calibration params
            self.declare_parameter('test_angle', 360.0)  # degrees
            self.declare_parameter('speed', 1.0)         # radians/sec
            self.declare_parameter('tolerance', 1.0)     # degrees
            self.declare_parameter('odom_angular_scale_correction', 1.0)

        self._state = "INIT"
        self._last_pose = None
        self._turn_angle = 0.0
        self._reverse = 1
        self._last_time = self.get_clock().now()

    def main_loop(self):
        # Avoid running before TF is ready
        if self._state == "INIT":
            try:
                self.tf_buffer.lookup_transform(
                    self.get_parameter('odom_frame').value,
                    self.get_parameter('base_frame').value,
                    rclpy.time.Time())
                self._state = "READY"
                self.get_logger().info("TF ready, calibration starting...")
                if self.mode == 'linear':
                    self._start_pos = self.get_position()
                else:
                    self._last_angle = self.get_yaw()
            except Exception as e:
                return

        if self._state != "READY":
            return

        # Check if test is started via parameter
        start_param = self.get_parameter('start_test') if 'start_test' in [p.name for p in self.list_parameters().names] else Parameter('start_test', Parameter.Type.BOOL, True)
        if not start_param.value:
            self.cmd_vel.publish(Twist())  # Stop
            return

        if self.mode == 'linear':
            self.linear_calibration()
        else:
            self.angular_calibration()

    def linear_calibration(self):
        test_distance = self.get_parameter('test_distance').value
        speed = self.get_parameter('speed').value
        tolerance = self.get_parameter('tolerance').value
        scale = self.get_parameter('odom_linear_scale_correction').value

        cur_pos = self.get_position()
        if cur_pos is None:
            return

        # Calculate Euclidean distance from start
        distance = sqrt(pow((cur_pos.x - self._start_pos.x), 2) +
                        pow((cur_pos.y - self._start_pos.y), 2))
        corrected_distance = distance * scale
        error = corrected_distance - test_distance

        move_cmd = Twist()
        if abs(error) < tolerance:
            self.cmd_vel.publish(Twist())
            self.get_logger().info(f"Linear calibration complete. "
                f"Distance: {distance:.3f}m (corrected: {corrected_distance:.3f}m). "
                f"Target: {test_distance}m. Correction factor: {scale}")
            self.set_parameters([Parameter('start_test', Parameter.Type.BOOL, False)])
        else:
            move_cmd.linear.x = copysign(speed, -error)
            self.cmd_vel.publish(move_cmd)

    def angular_calibration(self):
        test_angle = radians(self.get_parameter('test_angle').value)
        speed = self.get_parameter('speed').value
        tolerance = radians(self.get_parameter('tolerance').value)
        scale = self.get_parameter('odom_angular_scale_correction').value

        cur_angle = self.get_yaw()
        if cur_angle is None:
            return

        if not hasattr(self, "_odom_angle"):
            self._odom_angle = cur_angle
            self._turn_angle = 0
            self._reverse = 1
            self._last_angle = cur_angle

        delta_angle = scale * self.normalize_angle(cur_angle - self._last_angle)
        self._turn_angle += delta_angle
        error = test_angle * self._reverse - self._turn_angle

        move_cmd = Twist()
        if abs(error) < tolerance:
            self.cmd_vel.publish(Twist())
            self.get_logger().info(f"Angular calibration complete. "
                f"Turned: {degrees(self._turn_angle):.2f} deg (corrected). Target: {self.get_parameter('test_angle').value} deg. Correction: {scale}")
            self.set_parameters([Parameter('start_test', Parameter.Type.BOOL, False)])
            self._reverse *= -1
            self._turn_angle = 0
        else:
            move_cmd.angular.z = copysign(speed, error)
            self.cmd_vel.publish(move_cmd)

        self._last_angle = cur_angle

    def get_position(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.get_parameter('odom_frame').value,
                self.get_parameter('base_frame').value,
                rclpy.time.Time())
            trans = t.transform.translation
            return Point(x=trans.x, y=trans.y, z=trans.z)
        except Exception:
            return None

    def get_yaw(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.get_parameter('odom_frame').value,
                self.get_parameter('base_frame').value,
                rclpy.time.Time())
            rot = t.transform.rotation
            _, _, yaw = euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])
            return yaw
        except Exception:
            return None

    def normalize_angle(self, angle):
        while angle > pi:
            angle -= 2.0 * pi
        while angle < -pi:
            angle += 2.0 * pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = CalibrateMotion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.cmd_vel.publish(Twist())
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

