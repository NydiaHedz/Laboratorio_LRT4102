#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import atan2, radians, sqrt, cos, sin, pi

class MoveTurtlePDControl:
    def __init__(self):
        rospy.init_node('turtle_control_xyz')
        
        # Subscribe to turtle position topic
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)
        
        # Publish to turtle movement commands topic
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        
        # Message publication rate (10 Hz)
        self.rate = rospy.Rate(10)
        
        # Current position and orientation variables
        self.current_x = 0
        self.current_y = 0
        self.current_theta = 0
        
        # Previous errors for derivative control
        self.last_error_x = 0
        self.last_error_y = 0
        self.last_error_theta = 0

    def pose_callback(self, pose):
        # Function executed each time the turtle position is updated
        self.current_x = pose.x
        self.current_y = pose.y
        self.current_theta = pose.theta

    def move_turtle_to_desired_position(self, desired_x, desired_y, desired_theta=None):
        # Proportional and derivative constants (adjustable)
        Kp_linear = 0.8     # Reduced from 1.0
        Kd_linear = 0.5     # Increased for better damping
        Kp_angular = 2.0    # Reduced from 3.0
        Kd_angular = 0.8    # Increased for better damping
        
        # Thresholds to reduce oscillation
        distance_threshold = 0.2
        angle_threshold = radians(5)
        
        # Stop conditions (less strict to prevent oscillation)
        stop_distance = 0.15
        stop_angle = radians(5)
        
        # Minimum speed to prevent jittering at low speeds
        min_linear_speed = 0.05
        min_angular_speed = 0.1
        
        while not rospy.is_shutdown():
            # Calculate position errors
            error_x = desired_x - self.current_x
            error_y = desired_y - self.current_y
            
            # Calculate distance and angle to target
            distance = sqrt(error_x**2 + error_y**2)
            target_angle = atan2(error_y, error_x)
            
            # Calculate heading error (angle to target)
            error_theta = target_angle - self.current_theta
            
            # Normalize theta error to [-π, π]
            while error_theta > pi:
                error_theta -= 2*pi
            while error_theta < -pi:
                error_theta += 2*pi
            
            # Calculate derivative component for position and orientation errors
            d_error_linear = distance - sqrt(self.last_error_x**2 + self.last_error_y**2)
            d_error_theta = error_theta - self.last_error_theta
            
            # Save current errors for next iteration
            self.last_error_x = error_x
            self.last_error_y = error_y
            self.last_error_theta = error_theta
            
            # Create Twist message to send movement command
            twist_msg = Twist()
            
            # First correct orientation, then move forward
            if abs(error_theta) > angle_threshold:
                # If orientation error is large, just rotate and don't move forward
                twist_msg.linear.x = 0
                twist_msg.angular.z = Kp_angular * error_theta + Kd_angular * d_error_theta
                
                # Apply dead zone to prevent small oscillations
                if abs(twist_msg.angular.z) < min_angular_speed:
                    if twist_msg.angular.z > 0:
                        twist_msg.angular.z = min_angular_speed
                    else:
                        twist_msg.angular.z = -min_angular_speed
            else:
                # If orientation is approximately correct, move forward and make small angle corrections
                linear_speed = Kp_linear * distance + Kd_linear * d_error_linear
                angular_speed = Kp_angular * error_theta + Kd_angular * d_error_theta
                
                # Apply dead zone to prevent small oscillations
                if distance > distance_threshold:
                    twist_msg.linear.x = linear_speed
                    # Limit max speed
                    if twist_msg.linear.x > 1.5:
                        twist_msg.linear.x = 1.5
                else:
                    # As we get closer, reduce speed
                    twist_msg.linear.x = 0.5 * linear_speed
                    # Ensure minimum speed to overcome friction
                    if twist_msg.linear.x < min_linear_speed and twist_msg.linear.x > 0:
                        twist_msg.linear.x = min_linear_speed
                
                twist_msg.angular.z = angular_speed
                # Apply dead zone for angular motion
                if abs(twist_msg.angular.z) < min_angular_speed:
                    twist_msg.angular.z = 0
            
            # Publish message
            self.velocity_publisher.publish(twist_msg)
            
            # Print current position, errors and movement variables
            rospy.loginfo("Current: (%0.2f, %0.2f, %0.2f), Target: (%0.2f, %0.2f), Distance: %0.2f, Angle Error: %0.2f",
                         self.current_x, self.current_y, self.current_theta, 
                         desired_x, desired_y, distance, error_theta)
            
            # Check if desired position is reached
            if distance < stop_distance:
                # If specific orientation is requested, handle it
                if desired_theta is not None:
                    final_theta_error = desired_theta - self.current_theta
                    while final_theta_error > pi:
                        final_theta_error -= 2*pi
                    while final_theta_error < -pi:
                        final_theta_error += 2*pi
                        
                    if abs(final_theta_error) > stop_angle:
                        # Just rotate to desired orientation
                        twist_msg = Twist()
                        twist_msg.angular.z = Kp_angular * final_theta_error
                        self.velocity_publisher.publish(twist_msg)
                        continue
                
                # Stop the turtle
                twist_msg = Twist()
                self.velocity_publisher.publish(twist_msg)
                rospy.loginfo("Desired position reached")
                break
            
            # Wait until next iteration
            self.rate.sleep()

    def get_desired_position_from_user(self):
        print("Enter the desired position and orientation:")
        x = float(input("X coordinate: "))
        y = float(input("Y coordinate: "))
        theta_input = input("Theta (in degrees, press Enter to automatically face target): ")
        
        if theta_input.strip():
            theta = radians(float(theta_input))
            return x, y, theta
        else:
            return x, y, None

    def move_turtle_interactively(self):
        while not rospy.is_shutdown():
            try:
                # Get desired position from user
                desired_position = self.get_desired_position_from_user()
                
                # Move turtle to desired position
                if len(desired_position) == 3:
                    self.move_turtle_to_desired_position(desired_position[0], desired_position[1], desired_position[2])
                else:
                    self.move_turtle_to_desired_position(desired_position[0], desired_position[1])
            except ValueError as e:
                rospy.logerr("Invalid input: %s", str(e))

if __name__ == '__main__':
    try:
        move_turtle_pd = MoveTurtlePDControl()
        move_turtle_pd.move_turtle_interactively()
    except rospy.ROSInterruptException:
        pass
