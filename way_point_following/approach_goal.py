#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion

class ApproachGoal(Node):
    def __init__(self):
        super().__init__('approach_goal')
        # Declare parameters for the goal and controller gains.
        self.declare_parameter('goal_x', 0.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('linear_kp', 0.5)
        self.declare_parameter('angular_kp', 1.0)
        self.declare_parameter('distance_stop_threshold', 1.7)  # meters
        self.declare_parameter('angle_stop_threshold', 0.1745)    # ~10 degrees in radians

        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value
        self.linear_kp = self.get_parameter('linear_kp').value
        self.angular_kp = self.get_parameter('angular_kp').value
        self.distance_stop_threshold = self.get_parameter('distance_stop_threshold').value
        self.angle_stop_threshold = self.get_parameter('angle_stop_threshold').value

        # Publisher for cmd_vel
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        # Subscription to odometry (adjust topic if needed)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        # Current robot pose
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        # Timer for control loop at 10Hz
        self.timer = self.create_timer(0.1, self.control_loop)

    def odom_callback(self, msg):
        # Update the robot's current position and orientation.
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        (roll, pitch, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.current_yaw = yaw

    def control_loop(self):
        # Compute error between current pose and goal.
        error_x = self.goal_x - self.current_x
        error_y = self.goal_y - self.current_y
        distance_error = math.sqrt(error_x**2 + error_y**2)

        # Compute desired angle and angular error.
        desired_angle = math.atan2(error_y, error_x)
        angle_error = desired_angle - self.current_yaw
        # Normalize angle_error to be within [-pi, pi]
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        # Check if both conditions are met:
        # 1. distance error is less than distance_stop_threshold
        # 2. angular error is within angle_stop_threshold
        if distance_error < self.distance_stop_threshold and abs(angle_error) < self.angle_stop_threshold:
            # Publish zero velocities and log that the goal is reached.
            twist = Twist()
            self.cmd_pub.publish(twist)
            self.get_logger().info("Goal reached: Stopping robot.")
            return

        # Otherwise, compute control commands.
        twist = Twist()
        twist.linear.x = self.linear_kp * distance_error
        twist.angular.z = self.angular_kp * angle_error

        self.cmd_pub.publish(twist)
        self.get_logger().debug(f"Distance error: {distance_error:.2f}, Angle error: {angle_error:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = ApproachGoal()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
