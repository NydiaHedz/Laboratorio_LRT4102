# Laboratory_LRT4102
This repository contains the codes and reports for the laboratory sessions of the Robotics Systems Design course (LRT4102). 

## Labs Included  

### [Lab 1: Introduction to Python for Robotics](./Lab1/)
This first lab focused on developing problem-solving skills in Python. The exercises covered:  
- Basic syntax and control structures (loops, conditionals, functions).  
- Handling lists and dictionaries.  
- Object-Oriented Programming (OOP), including classes, inheritance, and encapsulation.  
- Implementing mathematical operations and algorithms.  
- Writing structured and efficient code for future robotics applications. 

The repository includes the solutions to the exercises, along with explanations and reports for each task.  

### [Lab 2: Fundamentals of ROS (Robot Operating System](./Lab2/)

This lab introduced core ROS concepts through hands-on practice with simulated robotics systems. The work covered three progressive levels:  

#### **1. Basic ROS Concepts**  
- Created a ROS package (`Practicas_lab`) with standard dependencies.  
- Implemented publisher-subscriber communication (`talker.py`/`listener.py`).  
- Analyzed node interactions using `rostopic` and `rqt_graph`.  

#### **2. Intermediate Robot Control**  
- Developed keyboard teleoperation systems for `turtlesim` (basic and enhanced versions).  
- Programmed autonomous trajectories to draw geometric shapes (square and triangle).  
- Utilized ROS services (`/kill`, `/spawn`, `/teleport_absolute`) for dynamic turtle management.  

#### **3. Advanced Control Strategies**  
- Designed and compared three controllers for `turtlesim`:  
  - **P** (Proportional): Fast but with steady-state error.  
  - **PD** (Proportional-Derivative): Stable with reduced oscillations.  
  - **PI** (Proportional-Integral): Precise but prone to overshoot.  
- Evaluated performance metrics (accuracy, settling time) using **PlotJuggler**.  
