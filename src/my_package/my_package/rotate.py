#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
import signal
import sys
import time

class TurningNode(Node):
    def __init__(self):
        super().__init__('turning_node')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_subscriber = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.angular_speed = math.radians(30)
        self.target_angle = None
        self.is_turning = False
        self.is_waiting = False
        self.turning_direction = 1
        self.initial_orientation = None
        self.sayac = 0

    def odom_callback(self, msg):
        if self.is_turning:
            current_angle = math.atan2(2 * (msg.pose.pose.orientation.z * msg.pose.pose.orientation.w),
                                        1 - 2 * ((msg.pose.pose.orientation.z ** 2) + (msg.pose.pose.orientation.y ** 2)))
            if self.target_angle is not None and abs(current_angle - self.target_angle) < math.radians(5):
                self.stop_turning()
                self.get_logger().info("Waiting for 3 seconds")
                time.sleep(3)
                self.get_logger().info("Turning direction...")
                self.is_waiting = False
                self.turn_direction()
             

    def turn(self, direction, angle):
        if not self.is_waiting:
            self.target_angle = direction * angle
            self.is_turning = True
            msg = Twist()
            msg.angular.z = direction * self.angular_speed
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing: "%s"' % msg)
            self.is_waiting = True
            self.sayac +=1
            if self.sayac == 4:
                self.sayac = 0

    def stop_turning(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
        self.get_logger().info('Turning stopped')
        self.is_turning = False

    def turn_direction(self):
        if self.sayac == 0:
            self.turn(1, math.radians(30))
            self.get_logger().info("Turning direction...")

        if self.sayac == 1:
            self.turn(-1, math.radians(0))
            self.get_logger().info("Turning direction...")

        if self.sayac == 2:
            self.turn(-1, math.radians(30))
            self.get_logger().info("Turning direction...")

        if self.sayac == 3:
            self.turn(1, math.radians(0))
            self.get_logger().info("Turning direction...")


        
    def save_initial_orientation(self, msg):
        if self.initial_orientation is None:
            self.initial_orientation = msg.pose.pose.orientation

    def get_current_orientation(self):
        return self.current_orientation

def signal_handler(sig, frame):
    print('Baybayyy!')
    stop_msg = Twist()
    stop_msg.angular.z = 0.0
    turning_node.publisher_.publish(stop_msg)
    rclpy.shutdown()
    sys.exit(0)

def main(args=None):
    rclpy.init(args=args)
    global turning_node
    turning_node = TurningNode()
    turning_node.turn_direction()

    signal.signal(signal.SIGINT, signal_handler)
    print('Press Ctrl+C to exit')
    rclpy.spin(turning_node)
    rclpy.shutdown()
    turning_node.destroy_node()

if __name__ == '__main__':
    main()

