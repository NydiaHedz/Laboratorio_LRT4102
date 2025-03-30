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
