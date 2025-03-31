#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import atan2, pi, sqrt

class TurtleProportionalController:
    def __init__(self):
        """Initialize ROS node and setup publishers/subscribers"""
        rospy.init_node('turtle_p_controller')
        
        # Pose subscriber and velocity publisher
        self.pose_sub = rospy.Subscriber('/turtle1/pose', Pose, self.pose_cb)
        self.vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        
        # Control rate (10Hz)
        self.rate = rospy.Rate(10)
        
        # Current pose storage
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.got_pose = False

    def pose_cb(self, msg):
        """Pose callback that updates current turtle position"""
        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta
        self.got_pose = True

    def move_to_goal(self, goal_x, goal_y, goal_theta):
        """
        Main control function that handles both position and orientation
        Prints remaining distance and angle error continuously
        """
        # Control gains
        Kp_lin = 0.8  # Linear proportional gain
        Kp_ang = 1.2  # Angular proportional gain
        
        # Tolerances
        pos_tol = 0.05  # Position tolerance (meters)
        ang_tol = 0.03  # Angle tolerance (radians)

        # Position control phase
        rospy.loginfo("\nStarting position control...")
        while not rospy.is_shutdown() and self.got_pose:
            # Calculate errors
            err_x = goal_x - self.x
            err_y = goal_y - self.y
            distance = sqrt(err_x**2 + err_y**2)
            
            # Print progress
            print(f"Remaining distance: {distance:.3f} m | X error: {err_x:.3f} | Y error: {err_y:.3f}", end='\r')
            
            # Check position tolerance
            if distance < pos_tol:
                print("\nPosition reached! Final error:", distance)
                break
            
            # Calculate and send velocity command
            cmd = Twist()
            cmd.linear.x = Kp_lin * err_x
            cmd.linear.y = Kp_lin * err_y
            self.vel_pub.publish(cmd)
            self.rate.sleep()

        # Orientation control phase
        rospy.loginfo("Starting orientation control...")
        while not rospy.is_shutdown() and self.got_pose:
            # Calculate angle error (normalized to [-π, π])
            err_theta = (goal_theta - self.theta + pi) % (2*pi) - pi
            
            # Print progress
            print(f"Angle error: {err_theta:.3f} rad | Current θ: {self.theta:.3f}", end='\r')
            
            # Check angle tolerance
            if abs(err_theta) < ang_tol:
                print("\nOrientation reached! Final error:", err_theta)
                break
            
            # Calculate and send rotation command
            cmd = Twist()
            cmd.angular.z = Kp_ang * err_theta
            self.vel_pub.publish(cmd)
            self.rate.sleep()

    def get_goal_from_user(self):
        """Get target coordinates from user input"""
        print("\n=== Enter Target Pose ===")
        try:
            x = float(input("X coordinate (1-10): "))
            y = float(input("Y coordinate (1-10): "))
            theta = float(input("Orientation (radians): "))
            return x, y, theta
        except ValueError:
            rospy.logwarn("Invalid input! Please enter numbers.")
            return None

    def run(self):
        """Main execution loop"""
        while not rospy.is_shutdown():
            # Wait for initial pose data
            while not self.got_pose and not rospy.is_shutdown():
                rospy.sleep(0.1)
            
            # Get target from user
            goal = self.get_goal_from_user()
            if goal is None:
                continue
            
            # Execute control
            self.move_to_goal(*goal)

if __name__ == '__main__':
    try:
        controller = TurtleProportionalController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
