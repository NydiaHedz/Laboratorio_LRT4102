# Laboratorio 3 

## Overview  

This project implements a proportional control navigation system for mobile robots based on **Euclidean geometry** fundamentals. The system calculates the straight-line distance to target (DTG) and required heading angle (ATG) to enable precise goal-oriented movement.  

## Core Mathematical Foundations  

### Euclidean Distance (DTG)  
The Euclidean distance between current position (x₁,y₁) and goal (x₂,y₂):  
\[
\text{DTG} = \sqrt{(x_2 - x_1)^2 + (y_2 - y_1)^2}
\]  
*Represents the shortest possible path to target in 2D space*

### Target Angle Calculation (ATG)  
The absolute target angle using the 2-argument arctangent:  
\[
\text{ATG} = \text{atan2}(y_2 - y_1, x_2 - x_1)
\]  

# TurtleBot Goal Spawner 

The `turtle_goal_spawner.py` script implements a ROS node that provides interactive control over a simulated turtle robot in the turtlesim environment. The system allows users to teleport the turtle to specified positions while calculating key navigation metrics. This report describes the functionality, implementation, and behavior of this system.

## Core Functionality  
The system operates through three main functions:  

1. **Dynamic Turtle Management**  
   The script can terminate the existing turtle and spawn a new one at user-specified coordinates through ROS services. This provides instant teleportation capability rather than physical movement.  

2. **Navigation Calculations**  
   Before any movement, the system computes two critical metrics:  
   - **Distance to Goal (DTG):** The straight-line distance calculated using the Euclidean distance formula  
   - **Angle to Goal (ATG):** The required heading angle determined using the atan2 function  

3. **User Interaction**  
   The script features an interactive console interface that prompts for destination coordinates and orientation. It includes input validation to ensure positions stay within the turtlesim's boundaries.

## Key Code Components  

### Pose Monitoring  
The script subscribes to the turtle's pose topic to continuously track its position:  
```python
pose = rospy.wait_for_message('/turtle1/pose', Pose, timeout=5)
```

### Navigation Calculations  
The core navigation formulas are implemented as:  
```python
def calculate_dtg(x1, y1, x2, y2):
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def calculate_atg(x1, y1, x2, y2):
    return math.degrees(math.atan2(y2 - y1, x2 - x1))
```

### Turtle Control Services  
The system uses standard turtlesim services for turtle management:  
```python
kill('turtle1')  # Terminates current turtle
spawn(x_new, y_new, math.radians(ang_new), 'turtle1')  # Creates new turtle
```

















# **TurtleBot Proportional Controller Report**  

The `turtle_controller_spawn.py` script implements a ROS-based proportional controller for navigating a simulated turtle robot to user-specified positions in the turtlesim environment. Unlike the spawner version that teleports the turtle, this solution employs closed-loop control to achieve smooth, physics-based movement while continuously displaying real-time navigation metrics.

## **System Architecture**  
The controller operates through three coordinated components:

### **1. Pose Monitoring System**  
A subscriber continuously tracks the turtle's position and orientation by listening to the `/turtle1/pose` topic. This real-time pose data serves as feedback for the control system. The callback stores the latest pose in the `self.pose` variable for use by other components.

### **2. Interactive User Interface**  
The system prompts users to enter target coordinates (x,y) and orientation (θ) through a console interface. Input validation ensures all positions stay within the valid turtlesim bounds (0-11 units) and handles numeric input errors gracefully. The interface displays the current pose before each new target request.

### **3. Two-Phase Proportional Controller**  
Movement occurs in two distinct phases with different control objectives:  

**Phase 1: Position Control**  
- Calculates Euclidean distance (DTG) and required heading angle (ATG)  
- Implements proportional control for both linear and angular velocity  
- Continuously displays real-time DTG and ATG values  
- Exits when within 0.1m of target position  

**Phase 2: Orientation Control**  
- Fine-tunes final orientation using angular proportional control  
- Operates silently without console output  
- Completes when within 0.1 radians of target angle  

## **Key Implementation Details**  

### **Control Algorithm**  
The proportional controller uses separate gains for position and orientation:  
```python
self.linear_kp = 1.5  # Position control gain
self.angular_kp = 4.0  # Orientation control gain
```

Velocity commands are computed as:  
```python
twist.linear.x = min(self.linear_kp * distance, 2.0)  # Capped at 2 m/s
twist.angular.z = self.angular_kp * angle_error
```

### **Navigation Calculations**  
The system computes fundamental navigation metrics:  
```python
distance = math.sqrt(dx**2 + dy**2)  # DTG
target_angle = math.atan2(dy, dx)    # ATG
angle_error = target_angle - self.pose.theta
```

### **Real-Time Feedback**  
Users receive continuous updates during movement:  
```python
print("\rDTG: %.2f m | ATG: %.2f°" % (distance, math.degrees(angle_error)), end="")
```
