"""
Explorer Robot Simulation

This program simulates a robot navigating through a randomly generated grid 
with obstacles. The robot starts at (0,0) and attempts to reach the bottom-right 
corner. It can only move forward, turn left, or turn right.

If the robot finds a way, the program displays:
1. The original grid with obstacles.
2. The path the robot followed using arrows.

If no path is found, it prints: "Impossible to reach the destination."

Author: Nydia Hernández Bravo
"""

import random

class ExplorerRobot:
    def __init__(self, size=5):
        self.size = size
        self.grid = [['o' for _ in range(size)] for _ in range(size)]
        self.robotPos = (0, 0)
        self.directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]  # Right, Down, Left, Up
        self.directionIndex = 0  # Initially facing right
        self.path = {}

    def generatePath(self):
        """Creates a guaranteed path from (0,0) to (size-1, size-1)"""
        x, y = 0, 0
        while (x, y) != (self.size - 1, self.size - 1):
            self.grid[x][y] = 'o'  # Mark as free space
            if x < self.size - 1 and random.choice([True, False]):  
                x += 1  # Move down
            elif y < self.size - 1:
                y += 1  # Move right
        self.grid[x][y] = 'o'  # Ensure exit is free

    def placeObstacles(self, numObstacles=None):
        """Randomly places obstacles without blocking the predefined path"""
        self.generatePath()  # Ensure there's a path before placing obstacles
        numObstacles = numObstacles if numObstacles is not None else self.size
        placed = 0
        while placed < numObstacles:
            x, y = random.randint(0, self.size - 1), random.randint(0, self.size - 1)
            if self.grid[x][y] == 'o' and (x, y) not in [(0, 0), (self.size-1, self.size-1)]:
                self.grid[x][y] = 'X'
                placed += 1

    def move(self):
        """Moves the robot forward if possible"""
        dx, dy = self.directions[self.directionIndex]
        new_x, new_y = self.robotPos[0] + dx, self.robotPos[1] + dy
        if 0 <= new_x < self.size and 0 <= new_y < self.size and self.grid[new_x][new_y] == 'o':
            self.robotPos = (new_x, new_y)
            self.path[self.robotPos] = self.getArrow()
            return True
        return False

    def turnLeft(self):
        """Turns the robot 90 degrees to the left"""
        self.directionIndex = (self.directionIndex - 1) % 4

    def turnRight(self):
        """Turns the robot 90 degrees to the right"""
        self.directionIndex = (self.directionIndex + 1) % 4

    def getArrow(self):
        """Returns the arrow corresponding to the robot's movement direction"""
        arrows = ['→', '↓', '←', '↑']
        return arrows[self.directionIndex]

    def explore(self):
        """Tries to reach the exit by exploring available paths"""
        attempts = 0
        while self.robotPos != (self.size-1, self.size-1):
            if self.move():
                continue
            self.turnRight()
            attempts += 1
            if attempts > 4:  # If the robot turns in a full circle without moving
                print("Impossible to reach the destination.")
                return False
        return True

    def printGrid(self):
        """Prints the grid with obstacles"""
        for i in range(self.size):
            for j in range(self.size):
                print(self.grid[i][j], end=' ')
            print()
        print()

    def printPath(self):
        """Prints the grid with the path followed by the robot"""
        for i in range(self.size):
            for j in range(self.size):
                if (i, j) == (0, 0):
                    print("S", end=" ")  # Start position
                elif (i, j) == (self.size-1, self.size-1):
                    print("E", end=" ")  # Exit position
                elif (i, j) in self.path:
                    print(self.path[(i, j)], end=" ")  # Path followed
                else:
                    print(self.grid[i][j], end=" ")  # Obstacles and free spaces
            print()
        print()

# Run the simulation
robot = ExplorerRobot(size=5)
robot.placeObstacles()
print("Initial Grid:")
robot.printGrid()

if robot.explore():
    print("Path Followed:")
    robot.printPath()
