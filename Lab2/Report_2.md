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
