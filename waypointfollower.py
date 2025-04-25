#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import pandas as pd
import numpy as np
from geometry_msgs.msg import Pose2D, Twist, Point, PoseWithCovarianceStamped
from visualization_msgs.msg import Marker, MarkerArray
from math import atan2, cos, sin, sqrt

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')

        self.waypoints = pd.read_csv('/home/f1tenth/F1tenthcpp/f1tenth_cpp_ws/maps/middleline.csv')
        self.waypoints = self.waypoints[['x', 'y', 'throttle']].values
        self.index = 0
        self.goal_tolerance = 0.5

        self.localized = False
        self.neutral_sent = False
        self.neutral_start_time = None
        self.pose_samples = []

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.marker_array_pub = self.create_publisher(MarkerArray, '/waypoints_marker_array', 10)
        self.pose_sub = self.create_subscription(Pose2D, '/pose2d', self.pose_callback, 10)


        self.timer = self.create_timer(0.1, self.control_loop)
        self.current_pose = None
        self.waypoint_publish_counter = 0

    def pose_callback(self, msg):
        self.current_pose = msg
        x = msg.x
        y = msg.y


        self.pose_samples.append((x, y))
        if len(self.pose_samples) > 10:
            self.pose_samples.pop(0)

        if not self.localized and len(self.pose_samples) == 10:
            origin = self.pose_samples[0]
            max_dist = max([sqrt((x - origin[0])**2 + (y - origin[1])**2) for (x, y) in self.pose_samples])
            if max_dist < 0.5:
                self.get_logger().info("✅ Pose2D is stable and clustered — localization confirmed.")
                self.localized = True
            else:
                self.get_logger().warn("⏳ Pose2D unstable — still waiting for consistent localization...")

    def control_loop(self):
        if self.waypoint_publish_counter < 20:
            self.publish_all_waypoints()
            self.waypoint_publish_counter += 1

        if not self.localized or self.current_pose is None:
            return

        if not self.neutral_sent:
            self.get_logger().info('🕒 Sending neutral PWM for ESC startup...')
            self.neutral_start_time = self.get_clock().now()
            self.neutral_sent = True

        if (self.get_clock().now() - self.neutral_start_time).nanoseconds < 5e9:
            neutral_cmd = Twist()
            neutral_cmd.linear.x = 0.0
            neutral_cmd.angular.z = 0.0
            self.cmd_pub.publish(neutral_cmd)
            return

        self.publish_all_waypoints()
        self.drive_to_waypoint()

    def drive_to_waypoint(self):
        if self.index >= len(self.waypoints):
            return

        px = self.current_pose.x
        py = self.current_pose.y
        yaw = self.current_pose.theta

        tx, ty, throttle = self.waypoints[self.index]
        dist = sqrt((tx - px) ** 2 + (ty - py) ** 2)

        if dist < self.goal_tolerance:
            self.index = (self.index + 1) % len(self.waypoints)
            return

        dx = tx - px
        dy = ty - py
        target_x = cos(yaw) * dx + sin(yaw) * dy
        target_y = -sin(yaw) * dx + cos(yaw) * dy
        angle_to_target = atan2(target_y, target_x)

        cmd = Twist()
        cmd.linear.x = float(throttle) / 100.0
        cmd.angular.z = angle_to_target
        self.cmd_pub.publish(cmd)

    def publish_all_waypoints(self):
        marker_array = MarkerArray()

        # Target waypoint marker (yellow)
        if self.index < len(self.waypoints):
            target = self.waypoints[self.index]
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = 0
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.pose.position.x = float(target[0])
            marker.pose.position.y = float(target[1])
            marker.pose.position.z = 0.1
            marker_array.markers.append(marker)

        # Car position marker (purple)
        if self.current_pose is not None:
            car_marker = Marker()
            car_marker.header.frame_id = 'map'
            car_marker.header.stamp = self.get_clock().now().to_msg()
            car_marker.id = 1
            car_marker.type = Marker.SPHERE
            car_marker.action = Marker.ADD
            car_marker.scale.x = 0.2
            car_marker.scale.y = 0.2
            car_marker.scale.z = 0.1
            car_marker.color.a = 1.0
            car_marker.color.r = 0.5
            car_marker.color.g = 0.0
            car_marker.color.b = 0.5
            car_marker.pose.position.x = self.current_pose.x
            car_marker.pose.position.y = self.current_pose.y
            car_marker.pose.position.z = 0.1
            marker_array.markers.append(car_marker)

        # All waypoints (colored by throttle)
        for i, waypoint in enumerate(self.waypoints):
            wp_marker = Marker()
            wp_marker.header.frame_id = 'map'
            wp_marker.header.stamp = self.get_clock().now().to_msg()
            wp_marker.id = i + 2
            wp_marker.type = Marker.SPHERE
            wp_marker.action = Marker.ADD
            wp_marker.scale.x = 0.1
            wp_marker.scale.y = 0.1
            wp_marker.scale.z = 0.05
            throttle = float(waypoint[2])
            wp_marker.color.a = 1.0
            if throttle <= 30:
                wp_marker.color.r = 0.0
                wp_marker.color.g = 1.0
                wp_marker.color.b = 0.0
            elif throttle <= 60:
                wp_marker.color.r = 1.0
                wp_marker.color.g = 1.0
                wp_marker.color.b = 0.0
            else:
                wp_marker.color.r = 1.0
                wp_marker.color.g = 0.0
                wp_marker.color.b = 0.0
            wp_marker.pose.position.x = float(waypoint[0])
            wp_marker.pose.position.y = float(waypoint[1])
            wp_marker.pose.position.z = 0.1
            marker_array.markers.append(wp_marker)

        self.marker_array_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
