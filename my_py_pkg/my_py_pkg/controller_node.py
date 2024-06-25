import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Quaternion
import math
import random


class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        self.subscriber = self.create_subscription(LaserScan, 'base_scan', self.laser_callback, 10)
        self.subscriber_odom = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.2, self.publish_movement)
        self.get_logger().info('Controller Node has been started')
        self.laser_msg = None
        self.odom = None
        self.goal_x = 17.3 #17.3
        self.goal_y = -0.3 #-0.3
        self.obstacle_mode = False
        self.rotating = False
        self.rotating_value = 0.0
        self.goal_reached_tolerance = 0.3  

    def laser_callback(self, msg):
        self.laser_msg = msg

    def odom_callback(self, msg):
        self.odom = msg

    def get_robot_orientation(self):
        if self.odom is None:
            return None
        orientation_q = self.odom.pose.pose.orientation
        return orientation_q

    def get_angle_to_goal(self, x, y, goal_x, goal_y):
        goal_angle = math.atan2(goal_y - y, goal_x - x)
        return goal_angle

    def quaternion_to_euler(self, quaternion):

        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x**2 + y**2))
        pitch = math.asin(2 * (w * y - z * x))
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))

        return roll, pitch, yaw

    def check_goal_reached(self, x, y):
        distance = math.sqrt((self.goal_x - x) ** 2 + (self.goal_y - y) ** 2)
        return distance < self.goal_reached_tolerance

    def update_goal(self):

        self.goal_x = 10.0 #10.0
        self.goal_y = -5.5 #-5.5
        self.get_logger().info(f'Goal reached. New goal: x={self.goal_x}, y={self.goal_y}')

    def publish_movement(self):
        if self.laser_msg is None or self.odom is None:
            return

        msg = Twist()
        x = self.odom.pose.pose.position.x
        y = self.odom.pose.pose.position.y

        if self.check_goal_reached(x, y):
            self.update_goal()

        robot_orientation = self.get_robot_orientation()
        goal_angle = self.get_angle_to_goal(x, y, self.goal_x, self.goal_y)

        if robot_orientation:
            roll, pitch, yaw = self.quaternion_to_euler(robot_orientation)

            angle_diff = goal_angle - yaw

            # Normalize angle_diff to be within [-pi, pi]
            angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))

            if abs(angle_diff) > 0.1 and not self.obstacle_mode:  # Rotate towards the goal
                msg.angular.z = 0.2 if angle_diff > 0 else -0.2
            elif any(self.laser_msg.ranges[i] < 1.0 for i in range(0, 270)):  # Obstacle detected
                self.obstacle_mode = True
                if any(self.laser_msg.ranges[i] < 1.0 for i in range(120, 150)):  # Obstacle in front
                    msg.linear.x = 0.0
                    if angle_diff > 0 and not self.rotating:
                        msg.angular.z = -0.25 
                        self.rotating = True
                        self.rotating_value = -0.25
                    elif angle_diff < 0 and not self.rotating:
                        msg.angular.z = 0.25
                        self.rotating = True
                        self.rotating_value = 0.25
                    elif self.rotating == True:
                        msg.angular.z = self.rotating_value
                    self.get_logger().info('Obstacle detected in front')
                elif any(self.laser_msg.ranges[i] < 1.0 for i in range(0, 121)):  # Obstacle on the right
                    msg.linear.x = 0.2
                    msg.angular.z = random.uniform(0.02, 0.04)
                    self.get_logger().info('Obstacle detected on right')
                elif any(self.laser_msg.ranges[i] < 1.0 for i in range(151, 270)):  # Obstacle on the left
                    msg.linear.x = 0.2
                    msg.angular.z = random.uniform(-0.02, -0.04)
                    self.get_logger().info('Obstacle detected on left')
            else:
                # No obstacle, move forward
                self.rotating = False
                self.rotating_value = 0.0
                self.obstacle_mode = False
                msg.linear.x = 0.2

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
