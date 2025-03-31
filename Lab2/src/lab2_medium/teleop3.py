#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.srv import Kill, Spawn, TeleportAbsolute
import math
import time

class TurtleArtist:
    def __init__(self):
        rospy.init_node('turtle_shape_drawer')
        self.cmd_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        
        # Wait for required services
        rospy.wait_for_service('/kill')
        rospy.wait_for_service('/spawn')
        rospy.wait_for_service('/turtle1/teleport_absolute')
        
        self.kill = rospy.ServiceProxy('/kill', Kill)
        self.spawn = rospy.ServiceProxy('/spawn', Spawn)
        self.teleport_turtle1 = rospy.ServiceProxy('/turtle1/teleport_absolute', TeleportAbsolute)

    def reset_turtle(self, name='turtle1'):
        """Position turtle at center facing right (0 degrees)"""
        rospy.loginfo(f"Resetting {name} position")
        teleport_service = rospy.ServiceProxy(f'/{name}/teleport_absolute', TeleportAbsolute)
        teleport_service(5.54, 5.54, 0)
        rospy.sleep(1)  # Ensure teleport completes

    def move_forward(self, distance, speed=1.0):
        """Move turtle forward exact distance"""
        rospy.loginfo(f"Moving forward {distance} meters")
        cmd = Twist()
        cmd.linear.x = speed
        duration = distance / speed
        start_time = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - start_time < duration:
            self.cmd_pub.publish(cmd)
            rospy.sleep(0.1)
        self.cmd_pub.publish(Twist())  # Stop
        rospy.sleep(0.5)

    def turn(self, angle, speed=1.0):
        """Turn turtle exact angle in radians"""
        rospy.loginfo(f"Turning {math.degrees(angle)} degrees")
        cmd = Twist()
        cmd.angular.z = speed if angle > 0 else -speed
        duration = abs(angle) / speed
        start_time = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - start_time < duration:
            self.cmd_pub.publish(cmd)
            rospy.sleep(0.1)
        self.cmd_pub.publish(Twist())  # Stop
        rospy.sleep(0.5)

    def draw_square(self):
        """Draw a square with 2m sides"""
        rospy.loginfo("Drawing square...")
        self.reset_turtle('turtle1')
        for _ in range(4):
            self.move_forward(2.0)
            self.turn(math.pi/2)  # 90 degrees
        rospy.loginfo("Square completed")

    def draw_triangle(self):
        """Draw an equilateral triangle with 2m sides"""
        rospy.loginfo("Drawing triangle...")
        self.reset_turtle('turtle2')
        for _ in range(3):
            self.move_forward(2.0)
            self.turn(2 * math.pi / 3)  # 120 degrees
        rospy.loginfo("Triangle completed")

    def run(self):
        try:
            # Draw square with turtle1
            self.draw_square()
            
            # Kill turtle1
            rospy.loginfo("Killing turtle1...")
            self.kill('turtle1')
            rospy.sleep(1)
            
            # Spawn turtle2
            rospy.loginfo("Spawning new turtle2...")
            self.spawn(5.54, 5.54, 0, 'turtle2')
            rospy.sleep(1)
            
            # Wait for teleport service of turtle2 to be available
            rospy.wait_for_service('/turtle2/teleport_absolute')
            
            # Update publisher for new turtle
            self.cmd_pub = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=10)
            rospy.sleep(1)
            
            # Draw triangle with turtle2
            self.draw_triangle()
            
            rospy.loginfo("All shapes drawn successfully!")
            
        except Exception as e:
            rospy.logerr(f"Error: {e}")

if __name__ == '__main__':
    artist = TurtleArtist()
    artist.run()

