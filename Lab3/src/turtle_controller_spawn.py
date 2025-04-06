#!/usr/bin/env python3

"""
turtle_controller_spawn.py 

Moves turtle to specified position/orientation while displaying DTG and ATG.
Uses proportional control for smooth movement.

Author: Nydia Hernandez Bravo
"""

import rospy
import math
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

class TurtleController:
    def __init__(self):
        rospy.init_node('turtle_controller')
        
        # Current pose subscriber
        self.pose = Pose()
        self.pose_sub = rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)
        
        # Velocity command publisher
        self.cmd_vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        
        # Controller parameters
        self.rate = rospy.Rate(10)  # 10Hz
        self.linear_kp = 1.5
        self.angular_kp = 4.0
        self.distance_tolerance = 0.1
        self.angle_tolerance = 0.1
        self.min_bound = 0.0
        self.max_bound = 11.0

    def pose_callback(self, msg):
        self.pose = msg

    def validate_position(self, x, y):
        """Check if position is within turtlesim bounds"""
        return (self.min_bound <= x <= self.max_bound and 
                self.min_bound <= y <= self.max_bound)

    def get_user_input(self):
        """Get and validate user input with re-prompting"""
        while not rospy.is_shutdown():
            try:
                print("\nCurrent Position: X=%.2f, Y=%.2f, θ=%.2f°" % 
                     (self.pose.x, self.pose.y, math.degrees(self.pose.theta)))
                x = float(input("Enter target X: "))
                y = float(input("Enter target Y: "))
                theta = float(input("Enter target angle (degrees): "))
                
                if not self.validate_position(x, y):
                    print("Error: Position must be between 0 and 11 for both X and Y")
                    continue
                    
                return x, y, math.radians(theta)
                
            except ValueError:
                print("Invalid input! Please enter numbers.")
                continue

    def move_to_target(self, target_x, target_y, target_theta):
        print("\nMoving to target... (Ctrl+C to cancel)")
        print("DTG = Distance to Goal | ATG = Angle to Goal")
        
        # Phase 1: Position control
        while not rospy.is_shutdown():
            dx = target_x - self.pose.x
            dy = target_y - self.pose.y
            distance = math.sqrt(dx**2 + dy**2)
            target_angle = math.atan2(dy, dx)
            angle_error = target_angle - self.pose.theta
            
            # Real-time DTG and ATG display
            print("\rDTG: %.2f m | ATG: %.2f°" % 
                 (distance, math.degrees(angle_error)), 
                 end="", flush=True)
            
            if distance < self.distance_tolerance:
                break
                
            twist = Twist()
            twist.linear.x = min(self.linear_kp * distance, 2.0)
            twist.angular.z = self.angular_kp * angle_error
            self.cmd_vel_pub.publish(twist)
            self.rate.sleep()
        
        # Phase 2: Orientation control (silent)
        while not rospy.is_shutdown():
            angle_error = target_theta - self.pose.theta
            if abs(angle_error) < self.angle_tolerance:
                break
                
            twist = Twist()
            twist.angular.z = self.angular_kp * angle_error
            self.cmd_vel_pub.publish(twist)
            self.rate.sleep()
        
        # Stop the turtle
        self.cmd_vel_pub.publish(Twist())
        print("\nTarget reached!")

    def run(self):
        while not rospy.is_shutdown():
            target = self.get_user_input()
            if target:
                self.move_to_target(*target)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = TurtleController()
        rospy.sleep(1)  # Wait for pose subscriber
        controller.run()
    except rospy.ROSInterruptException:
        pass
