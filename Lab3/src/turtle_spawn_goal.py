#!/usr/bin/env python3
"""
turtle_goal_spawner.py

Continuously kills current turtle and spawns new one at specified position/angle.
Calculates and displays DTG (Distance to Goal) and ATG (Angle to Goal).

Author: Nydia Hernandez Bravo
"""

import rospy
from turtlesim.srv import Kill, Spawn
from turtlesim.msg import Pose
import math

def calculate_dtg(x1, y1, x2, y2):
    """Calculate Distance to Goal (DTG)"""
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def calculate_atg(x1, y1, x2, y2):
    """Calculate Angle to Goal (ATG) in degrees"""
    dx = x2 - x1
    dy = y2 - y1
    return math.degrees(math.atan2(dy, dx))

def validate_position(x, y):
    """Check if position is within turtlesim bounds (0-11)"""
    return 0 <= x <= 11 and 0 <= y <= 11

def turtle_goal_spawner():
    rospy.init_node('turtle_goal_spawner')
    
    while not rospy.is_shutdown():
        try:
            # Get current position
            pose = rospy.wait_for_message('/turtle1/pose', Pose, timeout=5)
            x_current = pose.x
            y_current = pose.y
            ang_current = math.degrees(pose.theta)
            
            print("\n=== Turtle Position Controller ===")
            print(f"Current position: ({x_current:.2f}, {y_current:.2f})")
            print(f"Current angle: {ang_current:.2f}°")

            # Get user input with validation
            while True:
                try:
                    x_new = float(input("\nEnter X destination: "))
                    y_new = float(input("Enter Y destination: "))
                    ang_new = float(input("Enter orientation angle (degrees): "))
                    
                    if validate_position(x_new, y_new):
                        break
                    print("Error: Position must be between 0 and 11 for both X and Y")
                except ValueError:
                    print("Error: Please enter numeric values")

            # Calculate and display DTG and ATG
            dtg = calculate_dtg(x_current, y_current, x_new, y_new)
            atg = calculate_atg(x_current, y_current, x_new, y_new)
            
            print(f"\nDTG: {dtg:.2f} units")
            print(f"ATG: {atg:.2f}°")

            # Kill current turtle
            rospy.wait_for_service('/kill')
            try:
                kill = rospy.ServiceProxy('/kill', Kill)
                kill('turtle1')
            except rospy.ServiceException as e:
                print(f"Error killing turtle: {e}")
                continue

            # Spawn new turtle at desired position
            rospy.wait_for_service('/spawn')
            try:
                spawn = rospy.ServiceProxy('/spawn', Spawn)
                spawn(x_new, y_new, math.radians(ang_new), 'turtle1')
                print("\nNew turtle spawned successfully!")
                print(f"New position: ({x_new:.2f}, {y_new:.2f})")
                print(f"New angle: {ang_new:.2f}°")
            except rospy.ServiceException as e:
                print(f"Error spawning turtle: {e}")

        except rospy.ROSException:
            print("Error getting turtle pose. Retrying...")
            rospy.sleep(1)

if __name__ == '__main__':
    try:
        turtle_goal_spawner()
    except rospy.ROSInterruptException:
        print("\nShutting down turtle spawner...")
