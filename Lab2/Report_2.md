# Introduction  

This lab explored the fundamentals of ROS (Robot Operating System) through three levels of complexity: Basic, Medium, and Advanced. Each section had specific objectives to gain familiarity with package creation, node communication, simulated robot control, and performance analysis of different control strategies.  

1. **Basic Part**:  
   - A ROS package named `Practicas_lab` was created with dependencies on `rospy`, `roscpp`, and `std_msgs`.  
   - The `talker.py` and `listener.py` nodes were implemented to demonstrate publisher-subscriber communication.  
   - After compiling and running both nodes, their functionality and message interaction were analyzed.  

2. **Medium Part**:  
   - A keyboard-controlled teleoperation system was developed for the `turtlesim` robot.  
   - Additionally, autonomous trajectories were programmed to draw geometric shapes (a square and an equilateral triangle) without user intervention.  

3. **Advanced Part**:  
   - Three controllers were designed and implemented for `turtlesim`: **P** (Proportional), **PI** (Proportional-Integral), and **PID** (Proportional-Integral-Derivative).  
   - Their performance was compared using plotting tools like **PlotJuggler**, evaluating precision, settling time, and oscillations.  

This report details each stage, the obtained results, and conclusions on the behavior of the implemented systems. The code and generated plots are included as evidence of the work performed.  

## Basic Part

In this part two nodes were implemented in ROS (Robot Operating System): talker.py and listener.py. These nodes represent a fundamental example of inter-process communication using the publisher-subscriber paradigm, widely used in distributed robotic systems.

### Functional Description

#### Publisher Node (talker.py)
The talker.py node generates and periodically publishes messages to a specific ROS topic. It initializes a ROS node named "talker" and creates a publisher for the "chatter" topic with String-type messages. The publication frequency is set to 10 Hz using ROS's Rate object.

During execution, the node constructs messages combining "hello world" text with a timestamp obtained through rospy.get_time(). Each message is published to the designated topic while simultaneously being logged via rospy.loginfo(). The node includes exception handling to ensure clean termination upon receiving an interrupt signal.

#### Subscriber Node (listener.py)
The listener.py node consumes messages published to the "chatter" topic. After initializing as "listener", it subscribes to the mentioned topic, associating the callback function as the handler for incoming messages.

The callback function processes each received message, logging both the node identifier and message content via rospy.loginfo(). The node remains active indefinitely through rospy.spin(), maintaining execution while waiting for new messages.

#### Communication Architecture

The node communication follows ROS's typical publisher-subscriber model. The talker node publishes messages to the "chatter" topic, while the listener node subscribes to the same topic to receive these messages. The ROS Master acts as intermediary, managing node registration and message distribution.

To run the nodes:
1. Start ROS core:
   ```bash
   roscore
   ```
2. Run the publisher node:
   ```bash
   rosrun practicas_lab talker.py
   ```
3. Run the subscriber node:
   ```bash
   rosrun practicas_lab listener.py
   ```
The system's operational output when running these commands should appear as shown below:

![System's operational output](https://github.com/NydiaHedz/Laboratorio_LRT4102/blob/main/Lab2/media/lab2_basic.jpg)

The diagram shown bellow illustrates a fundamental publisher-subscriber pattern with two primary components: a publisher node named /tableer_3337_1743369893729 and a subscriber node named /listener_3355_1743369897472. These nodes communicate through the /chatter topic, which acts as the intermediary message channel. The numerical suffixes in the node names (1743369893729 and 1743369897472) indicate these were launched with anonymous=True parameter, generating unique identifiers to prevent naming conflicts.

The communication follows a unidirectional flow where the publisher node sends messages to the /chatter topic, which then distributes them to the subscriber node. This structure represents the canonical talker/listener example from ROS tutorials, typically using std_msgs/String messages.

![RQT graph](https://github.com/NydiaHedz/Laboratorio_LRT4102/blob/main/Lab2/media/lab2_basic_rqt.jpg)

### Conclusion

This system demonstrates basic principles of node communication in ROS, showing a fundamental pattern for developing distributed robotic systems. The presented implementation serves as a foundation for more complex scenarios requiring information exchange between robotic system components.

The used architecture allows easy extension to include multiple publishers or subscribers, more complex message processing, or hardware component integration. This example constitutes an essential starting point for developing more sophisticated robotic applications.

## Medium Part

The goal of thispart was to develop three progressively sophisticated control systems for the ROS Turtlesim simulator:

1. A basic keyboard teleoperation system 
2. An enhanced movement and rotation control system 
3. An autonomous geometric shape drawing system 

### Basic Keyboard Control

The complete code can be address in [teleop.py](https://github.com/NydiaHedz/Laboratorio_LRT4102/blob/main/Lab2/src/lab2_medium/teleop.py) 

This initial version provides fundamental movement control along the X and Y axes. The implementation uses a non-blocking keyboard input method to capture single key presses without requiring Enter. The control scheme is minimalist but functional:
- 'x' moves forward along X-axis (linear.x = 2.0)
- 'y' moves forward along Y-axis (linear.y = 2.0) 
- 's' stops all movement
- 'q' quits the program

#### Functional Description

The keyboard control system begins with terminal configuration for real-time input. The critical component is the `get_key()` function which handles low-level terminal settings:

```python
def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)  # Bypass line buffering
        key = sys.stdin.read(1)  # Single character read
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)  # Restore settings
    return key
```

This function temporarily switches the terminal to raw mode using `tty.setraw()`, reads exactly one keypress with `sys.stdin.read(1)`, then restores the original terminal configuration. The `finally` block ensures proper cleanup even if errors occur.

For ROS communication, the script establishes a publisher node:

```python
pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
```

This creates a channel to send `Twist` messages to the turtle's velocity topic. The `queue_size=10` parameter provides buffer space for message handling during high-frequency commands.

The movement logic translates keypresses into motion commands through a series of conditional statements:

```python
if key == 'x':
    msg.linear.x = 2.0  # Forward motion
elif key == 'y':
    msg.linear.y = 2.0  # Lateral motion
elif key == 's':
    msg.linear.x = msg.linear.y = 0.0  # Full stop
```

Each condition modifies different components of the `Twist` message. The X and Y axes are controlled independently, allowing diagonal movement when both are active simultaneously.

The main control loop ties everything together:

```python
while not rospy.is_shutdown():
    key = get_key()
    msg = Twist()
    # Key handling logic here
    pub.publish(msg)
    if key == 'q':
        break
```

This continuous loop reads inputs, prepares movement commands, and publishes them at each iteration. The `rospy.is_shutdown()` check ensures proper termination when ROS signals closure, while the 'q' key provides manual exit capability.

#### Launch File Configuration and Execution  

The launch file for this turtle teleoperation node is available in: [tortuga_teleop.launch](https://github.com/NydiaHedz/Laboratorio_LRT4102/blob/main/Lab2/src/launch/tortuga_teleop.launch)

To execute this package and activate keyboard control of the turtle in the Turtlesim environment, use the following command in your terminal:  

```bash
roslaunch practicas_lab tortuga_teleop.launch
```  

This command automatically performs the following steps:  
1. Starts the Turtlesim node (`turtlesim_node`)  
2. Executes the `teleop.py` script containing the control logic  
3. Establishes the necessary communication between nodes  

The system will be ready to receive keyboard commands following the predefined control scheme (keys **x**, **y**, **s** for movement and **q** to exit). The interface will display usage instructions in the terminal immediately after initialization.  

### Enhanced Movement System

The complete code is available in [teleop2.py](https://github.com/NydiaHedz/Laboratorio_LRT4102/blob/main/Lab2/src/lab2_medium/teleop2.py)

This upgraded version introduces omnidirectional movement and rotation control for the Turtlesim robot, using an intuitive `WASD`-style keyboard scheme. The implementation retains non-blocking input via `termios` and expands functionality with:  
- Linear motion: Forward (`w`), backward (`s`), left (`a`), right (`d`)  
- Angular motion: Counter-clockwise (`j`), clockwise (`l`)  
- Stop: Spacebar zeroes all velocities  
- Exit: `q` terminates the program  

#### Functional Description  

The `get_key()` function remains unchanged, ensuring real-time keypress detection without requiring Enter:  

```python
def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key
```  
The node (`turtle_keyboard_control`) publishes `Twist` messages to `/turtle1/cmd_vel`:  
```python
pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
```  
Keypresses map to specific `Twist` message components:  
- **Linear motion**:  
  ```python
  if key == 'w':
      msg.linear.x = 2.0   # Forward (X-axis)
  elif key == 's':
      msg.linear.x = -2.0  # Backward
  elif key == 'a':
      msg.linear.y = 2.0   # Left (Y-axis)
  elif key == 'd':
      msg.linear.y = -2.0  # Right
  ```  
Angular motion is performed with:  
  ```python
  elif key == 'j':
      msg.angular.z = 1.5   # Rotate left (positive Z)
  elif key == 'l':
      msg.angular.z = -1.5  # Rotate right
  ```  

The script prints clear instructions upon launch:  
```python
print("Control the turtle with keys:")
print("  w -> Forward")
print("  s -> Backward")
# ... (additional keybindings)
```  

#### Launch File Configuration and Execution  

The launch file for this turtle teleoperation node is available in: [tortuga_teleop2.launch](https://github.com/NydiaHedz/Laboratorio_LRT4102/blob/main/Lab2/src/launch/tortuga_teleop2.launch)

To execute this package and activate keyboard control of the turtle in the Turtlesim environment, use the following command in your terminal:  

```bash
roslaunch practicas_lab tortuga_teleop2.launch
```

The expected output is:  
- Turtlesim window spawns.  
- Terminal displays keybindings.  
- Robot responds to:  
  - `w`/`s`: Move forward/backward.  
  - `a`/`d`: Strafe left/right.  
  - `j`/`l`: Rotate in place.  
  - ` ` (space): Stop.  
  - `q`: Quit.  

#### Autonomous Shape Drawing

The complete code is available in [teleop3.py](https://github.com/NydiaHedz/Laboratorio_LRT4102/blob/main/Lab2/src/lab2_medium/teleop3.py) 

This advanced version automatically draws geometric shapes by:
1. Drawing a square with turtle1
2. Killing turtle1 via the /kill service
3. Spawning turtle2 via the /spawn service
4. Drawing a equilateral triangle with turtle2

#### Key Functionalities  

The `TurtleArtist` class initializes ROS services for turtle control:  
```python
self.kill = rospy.ServiceProxy('/kill', Kill)
self.spawn = rospy.ServiceProxy('/spawn', Spawn)
self.teleport_turtle1 = rospy.ServiceProxy('/turtle1/teleport_absolute', TeleportAbsolute)
```  
  Publishes linear velocity (`Twist.linear.x`) for a calculated duration (`distance / speed`).  
  ```python
  cmd.linear.x = speed
  duration = distance / speed
  ```  
  Publishes angular velocity (`Twist.angular.z`) for `abs(angle) / speed` seconds.  
  ```python
  cmd.angular.z = speed if angle > 0 else -speed
  ```  

The shape drawing methods are:

- Square: 4 iterations of `move_forward(2.0)` + `turn(math.pi/2)`.  
- Triangle: 3 iterations of `move_forward(2.0)` + `turn(2π/3)`.  

The turtle lifecycle management is performed with.

```python
self.kill('turtle1')                     # Remove turtle1
self.spawn(5.54, 5.54, 0, 'turtle2')     # Create turtle2
self.cmd_pub = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=10)  # Update publisher
```  

#### Execution Workflow  

1. **Initialize**:  
   - Start ROS node (`turtle_shape_drawer`).  
   - Wait for required services (`/kill`, `/spawn`, `/teleport_absolute`).  
2. **Draw Square**:  
   - Teleport `turtle1` to center.  
   - Execute square-drawing loop.  
3. **Switch Turtles**:  
   - Kill `turtle1`, spawn `turtle2` at the same position.  
4. **Draw Triangle**:  
   - Repeat motion primitives with 120° turns.  

#### Launch File Configuration and Execution  

The launch file for this turtle teleoperation node is available in: [tortuga_teleop3.launch](https://github.com/NydiaHedz/Laboratorio_LRT4102/blob/main/Lab2/src/launch/tortuga_teleop3.launch)

To execute this package and activate keyboard control of the turtle in the Turtlesim environment, use the following command in your terminal:  

```bash
roslaunch practicas_lab tortuga_teleop3.launch
```

An example of the result can be seen bellow

![Tutle draw](https://github.com/NydiaHedz/Laboratorio_LRT4102/blob/main/Lab2/media/lab2_medium.jpg)

### Conclusion

This project successfully implemented three distinct control paradigms for Turtlesim, each demonstrating important ROS concepts. Key achievements include:

1. Effective keyboard teleoperation implementation
2. Comprehensive movement and rotation control
3. Precise autonomous shape drawing through mathematical motion control
4. Proper integration of ROS services for turtle management

The main technical lessons learned were:
- The critical importance of precise timing in autonomous operations
- The need for careful velocity tuning for different movement types
- The value of service-based turtle management for complex behaviors
- How mathematical calculations translate to physical movements in simulation

These implementations provide a solid foundation for more advanced robotics control systems, demonstrating fundamental principles that scale to real-world applications. The project particularly highlights how simple command patterns can produce complex geometric behaviors through careful timing and motion control.

## Advanced Part 

This part describes the implementation and comparison of Proportional (P), Proportional-Integral (PI), and Proportional-Integral-Derivative (PID) controllers for position control in Turtlesim. The goal is to analyze their performance in terms of accuracy, response time, and stability using PlotJuggler for visualization.

### Position control for turtlesim (P)

This script implements a Proportional (P) controller for positioning a turtle in the ROS Turtlesim simulator. The code follows ROS conventions and demonstrates basic control theory implementation.  

The complete code can be seen in: [turtle_pc.py](https://github.com/NydiaHedz/Laboratorio_LRT4102/blob/main/Lab2/src/lab2_advanced/turtle_pc.py)

#### Functional Description  

The `TurtleProportionalController` class handles both position and orientation control through separate control loops:  

```python
class TurtleProportionalController:
    def __init__(self):
        rospy.init_node('turtle_p_controller')
        self.pose_sub = rospy.Subscriber('/turtle1/pose', Pose, self.pose_cb)
        self.vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)  # 10Hz control rate
        self.x, self.y, self.theta = 0.0, 0.0, 0.0
        self.got_pose = False
```

#### Key Components  

1. **Pose Callback**: Continuously updates the turtle's current state  
```python
def pose_cb(self, msg):
    self.x = msg.x
    self.y = msg.y
    self.theta = msg.theta
    self.got_pose = True
```

2. **Control Logic**: Implements two-stage P control  
```python
def move_to_goal(self, goal_x, goal_y, goal_theta):
    Kp_lin = 0.8  # Linear gain
    Kp_ang = 1.2  # Angular gain
    
    # Position control phase
    while not rospy.is_shutdown():
        err_x = goal_x - self.x
        err_y = goal_y - self.y
        distance = sqrt(err_x**2 + err_y**2)
        
        if distance < 0.05:  # 5cm tolerance
            break
            
        cmd = Twist()
        cmd.linear.x = Kp_lin * err_x
        cmd.linear.y = Kp_lin * err_y
        self.vel_pub.publish(cmd)
        self.rate.sleep()
    
    # Orientation control phase
    while not rospy.is_shutdown():
        err_theta = (goal_theta - self.theta + pi) % (2*pi) - pi
        
        if abs(err_theta) < 0.03:  # ~1.7° tolerance
            break
            
        cmd = Twist()
        cmd.angular.z = Kp_ang * err_theta
        self.vel_pub.publish(cmd)
        self.rate.sleep()
```

3. **User Interface**: Simple console input for target specification  
```python
def get_goal_from_user(self):
    print("\n=== Enter Target Pose ===")
    try:
        x = float(input("X coordinate (1-10): "))
        y = float(input("Y coordinate (1-10): "))
        theta = float(input("Orientation (radians): "))
        return x, y, theta
    except ValueError:
        rospy.logwarn("Invalid input! Please enter numbers.")
        return None
```

#### Launch File Configuration and Execution  

The launch file for this turtle teleoperation node is available in: [lab2_adv1.launch](https://github.com/NydiaHedz/Laboratorio_LRT4102/blob/main/Lab2/src/launch/lab2_adv1.launch)

To execute this package and activate keyboard control of the turtle in the Turtlesim environment, use the following command in your terminal:  

```bash
roslaunch practicas_lab lab2_adv1.launch
```

### Position control for turtlesim (PD)

The code implements a controller for positioning a turtle in the ROS Turtlesim simulator. The code follows ROS conventions and demonstrates advanced control theory implementation with both position and orientation control.

The complete code can be seen in [turtle_pdc.py](https://github.com/NydiaHedz/Laboratorio_LRT4102/blob/main/Lab2/src/lab2_advanced/turtle_pdc.py).

#### Functional Description

The `MoveTurtlePDControl` class handles position and orientation control through a comprehensive PD control loop that manages both simultaneously:

```python
class MoveTurtlePDControl:
    def __init__(self):
        rospy.init_node('turtle_control_xyz')
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)  # 10Hz control rate
        self.current_x, self.current_y, self.current_theta = 0.0, 0.0, 0.0
        self.last_error_x, self.last_error_y, self.last_error_theta = 0.0, 0.0, 0.0
```

#### Key Components

1. **Pose Callback**: Continuously updates the turtle's current state
```python
def pose_callback(self, pose):
    # Function executed each time the turtle position is updated
    self.current_x = pose.x
    self.current_y = pose.y
    self.current_theta = pose.theta
```

2. **Control Logic**: Implements a two-phase PD control with dead zones
```python
def move_turtle_to_desired_position(self, desired_x, desired_y, desired_theta=None):
    # Proportional and derivative constants (adjustable)
    Kp_linear = 0.8     
    Kd_linear = 0.5     
    Kp_angular = 2.0    
    Kd_angular = 0.8    
    
    # Thresholds to reduce oscillation
    distance_threshold = 0.2
    angle_threshold = radians(5)
    
    while not rospy.is_shutdown():
        # Calculate position errors
        error_x = desired_x - self.current_x
        error_y = desired_y - self.current_y
        
        # Calculate distance and angle to target
        distance = sqrt(error_x**2 + error_y**2)
        target_angle = atan2(error_y, error_x)
        
        # Calculate heading error and normalize to [-π, π]
        error_theta = target_angle - self.current_theta
        while error_theta > pi:
            error_theta -= 2*pi
        while error_theta < -pi:
            error_theta += 2*pi
        
        # Calculate derivative component for PD control
        d_error_linear = distance - sqrt(self.last_error_x**2 + self.last_error_y**2)
        d_error_theta = error_theta - self.last_error_theta
        
        # Save current errors for next iteration
        self.last_error_x = error_x
        self.last_error_y = error_y
        self.last_error_theta = error_theta
        
        # First correct orientation, then move forward
        if abs(error_theta) > angle_threshold:
            # Rotation-only phase
            twist_msg = Twist()
            twist_msg.angular.z = Kp_angular * error_theta + Kd_angular * d_error_theta
        else:
            # Combined movement phase
            twist_msg = Twist()
            twist_msg.linear.x = Kp_linear * distance + Kd_linear * d_error_linear
            twist_msg.angular.z = Kp_angular * error_theta + Kd_angular * d_error_theta
        
        # Publish velocity command
        self.velocity_publisher.publish(twist_msg)
```

3. **User Interface**: Console-based input for target specification
```python
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
```

1. **Dead Zone Implementation**: Prevents small oscillations near the target
```python
# Apply dead zone to prevent small oscillations
if abs(twist_msg.angular.z) < min_angular_speed:
    if twist_msg.angular.z > 0:
        twist_msg.angular.z = min_angular_speed
    else:
        twist_msg.angular.z = -min_angular_speed
```

2. **Progressive Speed Reduction**: Slows down as it approaches the target
```python
if distance > distance_threshold:
    twist_msg.linear.x = linear_speed
    # Limit max speed
    if twist_msg.linear.x > 1.5:
        twist_msg.linear.x = 1.5
else:
    # As we get closer, reduce speed
    twist_msg.linear.x = 0.5 * linear_speed
```

3. **Optional Target Orientation**: Can automatically face the target or use a specified orientation
```python
if desired_theta is None:
    # Auto-orientation towards target
    target_angle = atan2(error_y, error_x)
else:
    # Use specified orientation
    final_theta_error = desired_theta - self.current_theta
```
#### Launch File Configuration and Execution  

The launch file for this turtle teleoperation node is available in: [lab2_adv2.launch](https://github.com/NydiaHedz/Laboratorio_LRT4102/blob/main/Lab2/src/launch/lab2_adv2.launch)

To execute this package and activate keyboard control of the turtle in the Turtlesim environment, use the following command in your terminal:  

```bash
roslaunch practicas_lab lab2_adv2.launch

### Position control for turtlesim (PID)

The code implements a complete PID controller for position and orientation control of a turtle in ROS Turtlesim. The controller features interactive target input and handles both Cartesian position and angular orientation simultaneously.

The complete code can be adressed in: 

#### Functional Description

The `MoveTurtlePIDControl` class provides:

1. **Continuous pose tracking** via ROS subscriber
2. **PID control loop** running at 10Hz
3. **Interactive target input** through console
4. **Automatic coordinate transformation** from global to turtle frame

#### Key Components

1. **Control Loop Core**:
```python
# PID calculations for each axis
vel_x = Kp_x*error_x + Ki_x*integral_x + Kd_x*derivative_x
vel_θ = Kp_θ*error_θ + Ki_θ*integral_θ + Kd_θ*derivative_θ

# Transform to local frame
twist_msg.linear.x = vel_x * cos(θ) + vel_y * sin(θ)
twist_msg.angular.z = vel_θ
```

2. **Essential Functions**:
```python
def pose_callback(self, pose):
    """Updates current turtle pose"""
    self.current_x = pose.x
    self.current_y = pose.y
    self.current_θ = pose.θ

def normalize_angle(self, angle):
    """Keeps angles in [-π,π] range"""
    while angle > pi: angle -= 2*pi
    while angle < -pi: angle += 2*pi
    return angle
```

3. **User Interaction**:
```python
def get_target_from_user(self):
    """Gets target position/orientation from console"""
    x = float(input("Target X: "))
    y = float(input("Target Y: "))
    θ = input("θ (radians, Enter for auto): ")
    return x, y, θ if θ else None
```

#### Launch File Configuration and Execution  

The launch file for this turtle teleoperation node is available in: [lab2_adv3.launch](https://github.com/NydiaHedz/Laboratorio_LRT4102/blob/main/Lab2/src/launch/lab2_adv3.launch)

To execute this package and activate keyboard control of the turtle in the Turtlesim environment, use the following command in your terminal:  

```bash
roslaunch practicas_lab lab2_adv2.launch
```

### Comparison
**Control P**
![Controler P](https://github.com/NydiaHedz/Laboratorio_LRT4102/blob/main/Lab2/media/lab2_adv1.jpg)
**Control PD**
![Controler PD](https://github.com/NydiaHedz/Laboratorio_LRT4102/blob/main/Lab2/media/lab2_adv2.jpg)
**Control PI**
![Controler PI](https://github.com/NydiaHedz/Laboratorio_LRT4102/blob/main/Lab2/media/lab2_adv3.jpg)
