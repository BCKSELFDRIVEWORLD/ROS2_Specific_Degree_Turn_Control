#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import math
import time

class TurningNode(Node):
    def __init__(self):
        super().__init__('turning_node')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_subscriber = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.angular_speed = math.radians(30)
        self.target_angle = None
        self.is_turning = False
        self.is_waiting = Falses

        self.declare_parameter('start_position', 0.0)
        self.start_position = self.get_parameter('start_position').get_parameter_value().double_value

    def odom_callback(self, msg):
        if self.is_turning:
            current_angle = math.atan2(2 * (msg.pose.pose.orientation.z * msg.pose.pose.orientation.w),
                                        1 - 2 * (msg.pose.pose.orientation.z ** 2))

            if self.target_angle is not None and abs(current_angle - self.target_angle) < math.radians(5):
                self.stop_turning()
                self.get_logger().info("Waiting for 3 seconds")
                time.sleep(3)
                self.is_waiting = False
                self.turn_right()
                self.start_position = current_angle

                self.set_parameters([rclpy.parameter.Parameter('start_position', rclpy.Parameter.Type.DOUBLE, self.start_position)])

    def turn(self, direction, angle):
        if not self.is_waiting:
            self.target_angle = direction * angle
            self.is_turning = True
            msg = Twist()
            msg.angular.z = direction * self.angular_speed
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing: "%s"' % msg)
            self.is_waiting = True

    def stop_turning(self):
        msg = Twist()
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
        self.get_logger().info('Turning stopped')
        self.is_turning = False

    def turn_left(self):
        self.turn(1, math.radians(30))

    def turn_right(self):
        self.turn(-1, math.radians(30))

def main(args=None):
    rclpy.init(args=args)
    turning_node = TurningNode()


    while True:
        for _ in range(100):
            turning_node.turn_left()
            try:
                rclpy.spin_once(turning_node, timeout_sec=1.0)
            except KeyboardInterrupt:
                pass

        turning_node.destroy_node()
    
        time.sleep(3)  
        rclpy.init(args=args)  
        turning_node = TurningNode()

        for _ in range(100):
            turning_node.turn_right()
            try:
                rclpy.spin_once(turning_node, timeout_sec=1.0)
            except KeyboardInterrupt:
                pass

        turning_node.destroy_node()

if __name__ == '__main__':
    main()
