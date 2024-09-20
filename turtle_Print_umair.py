#!/usr/bin/env python3
# ros2 run turtlesim turtlesim_node # make this subscriber node run first
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from time import sleep

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_control')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.velocity = Twist()

    def move_straight(self, speed, duration):
        self.velocity.linear.x = speed
        self.velocity.angular.z = 0.0
        self.publish_for_duration(duration)

    def rotate(self, angular_speed, duration):
        self.velocity.linear.x = 0.0
        self.velocity.angular.z = angular_speed
        self.publish_for_duration(duration)

    def stop(self):
        self.velocity.linear.x = 0.0
        self.velocity.angular.z = 0.0
        self.publisher_.publish(self.velocity)

    def publish_for_duration(self, duration):
        for _ in range(int(duration * 10)):  # publish at 10Hz
            self.publisher_.publish(self.velocity)
            sleep(0.1)

    def write_umair(self):
        # Drawing U
        self.move_straight(2.0, 2.0)  # Move up
        self.rotate(1.57, 1.0)        # Turn right 90 degrees
        self.move_straight(1.0, 2.0)  # Move right
        self.rotate(1.57, 1.0)        # Turn right 90 degrees
        self.move_straight(2.0, 2.0)  # Move down

        # Drawing M
        self.rotate(-1.57, 1.0)       # Turn left 90 degrees
        self.move_straight(2.0, 2.0)  # Move up
        self.rotate(-2.36, 1.0)       # Turn left diagonally
        self.move_straight(1.5, 1.0)  # Move diagonally down-right
        self.rotate(2.36, 1.0)        # Turn right diagonally
        self.move_straight(1.5, 1.0)  # Move diagonally up-right
        self.rotate(1.57, 1.0)        # Turn right
        self.move_straight(2.0, 2.0)  # Move down

        # Drawing A
        self.rotate(-1.57, 1.0)       # Turn left 90 degrees
        self.move_straight(2.0, 2.0)  # Move up
        self.rotate(-2.36, 1.0)       # Turn left diagonally
        self.move_straight(2.0, 1.0)  # Move diagonally down-right
        self.rotate(2.36, 1.0)        # Turn right diagonally
        self.move_straight(1.0, 1.0)  # Move diagonally up-right
        self.rotate(3.14, 1.0)        # Turn 180 degrees
        self.move_straight(1.0, 1.0)  # Move across A's bar

        # Drawing I
        self.rotate(-1.57, 1.0)       # Turn left 90 degrees
        self.move_straight(2.0, 2.0)  # Move up
        self.move_straight(-2.0, 2.0) # Move down

        # Drawing R
        self.rotate(-1.57, 1.0)       # Turn left 90 degrees
        self.move_straight(2.0, 2.0)  # Move up
        self.rotate(1.57, 1.0)        # Turn right 90 degrees
        self.move_straight(1.0, 1.0)  # Move right
        self.rotate(1.57, 1.0)        # Turn right 90 degrees
        self.move_straight(1.0, 1.0)  # Move down
        self.rotate(1.57, 1.0)        # Turn right 90 degrees
        self.move_straight(1.0, 1.0)  # Move right
        self.rotate(3.14, 1.0)        # Turn 180 degrees
        self.move_straight(1.5, 1.0)  # Move diagonally down-right

    def run(self):
        self.write_umair()
        self.stop()

def main(args=None):
    rclpy.init(args=args)
    turtle_controller = TurtleController()
    turtle_controller.run()
    turtle_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
