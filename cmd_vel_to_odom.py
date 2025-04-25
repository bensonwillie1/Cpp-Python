#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
import tf_transformations
import tf2_ros
import math

class CmdVelToOdom(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_odom')

        self.WHEELBASE = 0.32  # meters
        self.steering_angle = 0.0
        self.vx = 0.0
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        self.create_subscription(Float32, '/servo_pwm', self.servo_callback, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        self.publisher_ = self.create_publisher(Odometry, '/odom', 10)
        self.br = tf2_ros.TransformBroadcaster(self)

        self.timer = self.create_timer(0.05, self.update_odom)

    def servo_callback(self, msg):
        pwm = msg.data
        # Convert PWM duty cycle (3-9) into steering angle in radians
        # 6 = center, range is [-0.4, +0.4] rad
        self.steering_angle = (6.0 - pwm) * (0.4 / 3.0)

    def cmd_vel_callback(self, msg):
        self.vx = msg.linear.x  # Don't use angular.z — calculated via steering

    def update_odom(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # Compute angular velocity from Ackermann kinematics
        if abs(self.steering_angle) > 1e-3:
            vth = self.vx / self.WHEELBASE * math.tan(self.steering_angle)
        else:
            vth = 0.0

        delta_x = self.vx * math.cos(self.theta) * dt
        delta_y = self.vx * math.sin(self.theta) * dt
        delta_th = vth * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_th

        q = tf_transformations.quaternion_from_euler(0, 0, self.theta)

        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.angular.z = vth

        # Covariance
        if abs(self.vx) > 0.01 or abs(vth) > 0.01:
            linear_cov = 0.02
            angular_cov = 0.05
        else:
            linear_cov = 9999.0
            angular_cov = 9999.0

        odom.pose.covariance = [9999.0] * 36
        odom.twist.covariance = [
            linear_cov, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 9999.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 9999.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 9999.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 9999.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, angular_cov
        ]

        self.publisher_.publish(odom)

        # Broadcast TF
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.br.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
