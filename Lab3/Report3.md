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
