# Lab 1
## Introduction to Python

Python is a versatile, high-level programming language known for its readability and extensive libraries. It's widely used in web development, data analysis, artificial intelligence, and more. [1]

### Data Types in Python

Python offers a variety of built-in data types to handle different kinds of data:

- **Numeric Types**: `int` (integer), `float` (floating-point number), and `complex` (complex number).
- **Sequence Types**: `list` (ordered, mutable sequence), `tuple` (ordered, immutable sequence), and `range` (sequence of numbers).
- **Mapping Type**: `dict` (dictionary, a collection of key-value pairs).
- **Set Types**: `set` and `frozenset` (unordered collections of unique elements).
- **Boolean Type**: `bool` (represents `True` or `False`).
- **Text Type**: `str` (string, a sequence of Unicode characters).

Understanding these data types is crucial for effective programming in Python. [2]

### Control Structures in Python

Control structures manage the flow of a program. Python supports several control structures, including:

- **Conditional Statements**: Using `if`, `elif`, and `else` to execute code based on conditions.

  ```python
  x = 10
  if x > 0:
      print("Positive")
  elif x == 0:
      print("Zero")
  else:
      print("Negative")
  ```

- **For Loops**: Iterate over a sequence (like a list or string).

  ```python
  for i in range(5):
      print(i)
  ```

- **While Loops**: Repeat as long as a condition is true.

  ```python
  count = 0
  while count < 5:
      print(count)
      count += 1
  ```

These structures are fundamental for directing the execution flow in Python programs. [3]

### Object-Oriented Programming (OOP) in Python

Python fully supports object-oriented programming (OOP), a paradigm centered around objects and classes. The core concepts include:

- **Classes and Objects**: A class is a blueprint for creating objects (instances). Objects are instances of classes.

  ```python
  class Dog:
      def __init__(self, name):
          self.name = name

      def bark(self):
          print(f"{self.name} says woof!")

  my_dog = Dog("Buddy")
  my_dog.bark()  # Output: Buddy says woof!
  ```

- **Inheritance**: A mechanism where a new class derives attributes and methods from an existing class.

  ```python
  class Animal:
      def __init__(self, name):
          self.name = name

      def speak(self):
          print(f"{self.name} makes a sound.")

  class Cat(Animal):
      def speak(self):
          print(f"{self.name} says meow!")

  my_cat = Cat("Whiskers")
  my_cat.speak()  # Output: Whiskers says meow!
  ```

- **Encapsulation**: Bundling data and methods that operate on that data within a single unit or class.

  ```python
  class Counter:
      def __init__(self):
          self.__count = 0  # Private attribute

      def increment(self):
          self.__count += 1

      def get_count(self):
          return self.__count

  counter = Counter()
  counter.increment()
  print(counter.get_count())  # Output: 1
  ```

- **Polymorphism**: The ability to present the same interface for different underlying data types.

  ```python
  class Shape:
      def area(self):
          pass

  class Rectangle(Shape):
      def __init__(self, width, height):
          self.width = width
          self.height = height

      def area(self):
          return self.width * self.height

  class Circle(Shape):
      def __init__(self, radius):
          self.radius = radius

      def area(self):
          import math
          return math.pi * self.radius ** 2

  shapes = [Rectangle(3, 4), Circle(2)]
  for shape in shapes:
      print(shape.area())
  ```

Embracing these OOP principles allows for the creation of modular, reusable, and organized code in Python. [4]

### References

1. Python Introduction. (n.d.). GeeksforGeeks. Retrieved October 1, 2023, from https://www.geeksforgeeks.org/introduction-to-python/
2. Python Data Types. (n.d.). GeeksforGeeks. Retrieved October 1, 2023, from https://www.geeksforgeeks.org/python-data-types/
3. Python Basics: Syntax, Data Types, and Control Structures. (2023, September 2). KDnuggets. https://www.kdnuggets.com/python-basics-syntax-data-types-and-control-structures
4. Object-Oriented Programming (OOP) in Python. (n.d.). Real Python. Retrieved October 1, 2023, from https://realpython.com/python3-object-oriented-programming/

## Solved Problems

### Problem 1: Sum of Positive Integers

Write a program that reads a positive integer "n" entered by the user and then displays  
the sum of all integers from 1 to n.

The sum of the first n positive integers can be calculated as follows:

S = n(n + 1) / 2

#### Solution
The implemented solution was:

```python
# Ask the user for a positive integer
n = int(input("Enter an integer: "))

# Check if the number is positive
if n > 0:
    # Calculate the sum using the mathematical formula
    sum_n = (n * (n + 1)) / 2  
    print(f"The sum of the first {n} integers is: {sum_n}")
else:
    print("Please enter a positive integer.")
```

This program calculates the sum of the first **n** positive integers using a simple formula. It first asks the user to enter a positive number and checks if the input is valid. If the number is positive, it applies the formula S = n(n + 1) / 2 to quickly compute the sum without using loops.

If the user enters a non-positive number, the program displays a message asking for a valid input. 

### Problem 2: Salary Calculation

Write a program that asks the user for the number of hours worked and the hourly wage. Then, display the corresponding payment on the screen.

#### Solution
The implemented solution was:

```python
# Ask the user for the number of hours worked and the hourly rate
hoursWorked = float(input("Enter the number of hours worked: "))
hourlyRate = float(input("Enter the hourly rate: "))

# Calculate the payment
payment = hoursWorked * hourlyRate

# Display the result
print(f"The payment for {hoursWorked} hours worked at a rate of {hourlyRate} per hour is: ${payment}")
```

The program calculates the total payment by multiplying the hours worked by the hourly rate. It takes user input for both values, converts them to `float`, and performs the calculation. The result is then displayed using an `f-string`, showing the hours, rate, and total payment in a clear format.

### Problem 3: Payroll Calculation for Multiple Operators

Create a list with at least six operators, each containing:
- Name  
- Hourly wage  
- Hours worked  

The program should calculate and print the total payment for each operator.

#### Soution
The implemented solution was:

```python
# List of operators with (Name, Hourly Wage, Hours Worked)
operatorList = [
    ("Joan", 15, 40),
    ("Miguel", 18.0, 35),
    ("Pablo", 20.0, 42),
    ("Emilio", 17.5, 38),
    ("Jesus", 16.0, 45),
    ("Valeria", 29.0, 37)
]

# Iterate through the list and calculate the total payment for each operator
for name, hourlyWage, hoursWorked in operatorList:
    totalPay = hourlyWage * hoursWorked
    print(f"{name} will be paid: ${totalPay:.2f}")
```

The program stores a list of six operators, where each entry contains a name, hourly wage, and hours worked. It then iterates through the list, calculating the total payment for each operator using: Total Payment = Hourly Wage * Hours Worked

Each operator’s payment is printed using an `f-string`, ensuring the output is formatted to two decimal places for clarity.

### Problem 4: List Operations

- Create a list called `numbers` containing at least 10 numbers.  
- Calculate the average of the even numbers.  
- Calculate the product of the odd numbers.  
- Print the results.

#### Soution
The implemented solution was:

```python
# Create a list with at least 10 numbers
numeros = [19, 20, 15, 22, 9, 10, 5, 18, 11, 6]

# Separate even and odd numbers
evenNumbers = [num for num in numeros if num % 2 == 0]
oddNumbers = [num for num in numeros if num % 2 != 0]

# Calculate the average of even numbers
averageEven = sum(evenNumbers) / len(evenNumbers) if evenNumbers else 0

# Calculate the product of odd numbers
productOdd = 1
for num in oddNumbers:
    productOdd *= num

# Print the results
print(f"Average of even numbers: {averageEven:.2f}")
print(f"Product of odd numbers: {productOdd}")
```

The program creates a list of 10 numbers and separates them into even and odd numbers using list comprehensions. It then calculates the average of even numbers by dividing their sum by the count, ensuring it doesn't divide by zero. The product of odd numbers is found by multiplying all values in the list. Finally, both results are printed with the average formatted to two decimal places.

### Problem 5: Guess the Secret Number

Create a program that asks the user to guess a secret number.  

- The program should generate a random number between 1 and 10.  
- The user must keep guessing until they find the correct number.  
- The program should provide hints if the guessed number is too high or too low.  
- The `while` loop should continue until the user guesses correctly.  
- At the end, print the number of attempts it took to guess the number.  

#### Soution
The implemented solution was:

```python
import random

# Generate a random number between 1 and 10
secretNumber = random.randint(1, 10)

# Initialize the attempt counter
attempts = 0

# Loop until the user guesses correctly
while True:
    # Get user's guess
    userGuess = int(input("Guess a number between 1 and 10: "))
    attempts += 1

    # Check the guess
    if userGuess < secretNumber:
        print("Too low! Try again.")
    elif userGuess > secretNumber:
        print("Too high! Try again.")
    else:
        print(f"Congratulations! You guessed the number after {attempts} attempts.")
        break
```

The program generates a random number between 1 and 10 using `random.randint(1, 10)`. It then enters a `while True` loop, continuously asking the user to guess the number. Each guess increases the attempt counter.

The program checks if the guessed number is too low, too high, or correct:
- If the guess is lower than the secret number, it prints "Too low! Try again."
- If the guess is higher, it prints "Too high! Try again."
- If the guess is correct, it prints a congratulatory message along with the number of attempts, then exits the loop using `break`.

### Problem 6: Explorer Robot

Create a program that generates a matrix of at least 5x5.  

- The robot starts at position (0,0) and must reach position (4,4) or the maximum position if the matrix size changes.  
- The number and position of obstacles are random.  
- The robot can only move forward, turn left, or turn right to find a free path.  
- If the robot cannot reach the destination, print "Impossible to reach the destination."  
- If the robot reaches the final position, print a map where:  
  - X represents an obstacle.  
  - o represents a free space.  

Example Output
```
o o o X o
o o o o o
o o o o X
o o o o o
o X X X o
```

- The program must also print a second map showing the path taken by the robot, using arrows:  

  - ↑ (U+2191) for up  
  - ↓ (U+2193) for down  
  - ← (U+2190) for left  
  - → (U+2192) for right  

#### Soution
The implemented solution can be seen in: [View the code](./src/P6.py)


This program simulates a robot navigating through a grid while avoiding obstacles. The robot starts at (0,0) and attempts to reach (size-1, size-1). It can move forward, turn left, or turn right. If it cannot find a path, the program prints "Impossible to reach the destination."

**1. Initializing the Robot**
```python
class ExplorerRobot:
    def __init__(self, size=5):
        self.size = size
        self.grid = [['o' for _ in range(size)] for _ in range(size)]
        self.robotPos = (0, 0)
        self.directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]  # Right, Down, Left, Up
        self.directionIndex = 0  # Initially facing right
        self.path = {}
```
- Creates a grid filled with `'o'` (open spaces).  
- The robot starts at (0,0) and initially faces right.  
- Defines movement directions (right, down, left, up).  

**2. Generating a Path**
```python
def generatePath(self):
    x, y = 0, 0
    while (x, y) != (self.size - 1, self.size - 1):
        self.grid[x][y] = 'o'
        if x < self.size - 1 and random.choice([True, False]):  
            x += 1  
        elif y < self.size - 1:
            y += 1  
    self.grid[x][y] = 'o'
```
- Creates a guaranteed path from the start to the exit.  
- Moves randomly right or down to ensure connectivity.  

**3. Placing Obstacles**
```python
def placeObstacles(self, numObstacles=None):
    self.generatePath()
    numObstacles = numObstacles if numObstacles is not None else self.size
    placed = 0
    while placed < numObstacles:
        x, y = random.randint(0, self.size - 1), random.randint(0, self.size - 1)
        if self.grid[x][y] == 'o' and (x, y) not in [(0, 0), (self.size-1, self.size-1)]:
            self.grid[x][y] = 'X'
            placed += 1
```
- Randomly places obstacles (`'X'`) without covering the start or exit.  

**4. Moving the Robot**
```python
def move(self):
    dx, dy = self.directions[self.directionIndex]
    new_x, new_y = self.robotPos[0] + dx, self.robotPos[1] + dy
    if 0 <= new_x < self.size and 0 <= new_y < self.size and self.grid[new_x][new_y] == 'o':
        self.robotPos = (new_x, new_y)
        self.path[self.robotPos] = self.getArrow()
        return True
    return False
```
- Moves forward if the path is open.  
- Updates position and records movement direction.  

**5. Turning Left or Right**
```python
def turnLeft(self):
    self.directionIndex = (self.directionIndex - 1) % 4

def turnRight(self):
    self.directionIndex = (self.directionIndex + 1) % 4
```
- Left turn: Moves counterclockwise.  
- Right turn: Moves clockwise.  

**6. Exploring the Grid**
```python
def explore(self):
    attempts = 0
    while self.robotPos != (self.size-1, self.size-1):
        if self.move():
            continue
        self.turnRight()
        attempts += 1
        if attempts > 4:
            print("Impossible to reach the destination.")
            return False
    return True
```
- Moves until reaching the exit.  
- Turns right if blocked, trying all directions.  
- If no moves are possible, prints failure message.  

**7. Printing the Grid and Path**
```python
def printGrid(self):
    for i in range(self.size):
        for j in range(self.size):
            print(self.grid[i][j], end=' ')
        print()
    print()
```
- Displays the grid with obstacles.

```python
def printPath(self):
    for i in range(self.size):
        for j in range(self.size):
            if (i, j) == (0, 0):
                print("S", end=" ")
            elif (i, j) == (self.size-1, self.size-1):
                print("E", end=" ")
            elif (i, j) in self.path:
                print(self.path[(i, j)], end=" ")
            else:
                print(self.grid[i][j], end=" ")
        print()
    print()
```
- Displays the path followed by the robot**, using:  
  - `S` for start position  
  - `E` for exit position  
  - `→ ↓ ← ↑` for robot movements  

**8. Running the Simulation**
```python
robot = ExplorerRobot(size=5)
robot.placeObstacles()
print("Initial Grid:")
robot.printGrid()

if robot.explore():
    print("Path Followed:")
    robot.printPath()
```
- Creates a 5x5 grid with obstacles.  
- Attempts to find a path and prints the results.  

### Exercise 7: Store Inventory Management

A store wants to manage its product inventory. Implement a Python system that allows:

- Creating products, each with a name, price, and stock quantity.  
- Updating the stock quantity when products are sold.  
- Displaying product information, including its availability.  
- Calculating the total inventory value using the formula: Total Value = Price * Stock Quantity

#### Soution
The implemented solution can be seen in: [View the code](./src/P7.py)

This program simulates a basic inventory system. It allows managing products by tracking their name, price, and stock quantity. The system also includes functionality to handle sales, display product details, and calculate the total inventory value. 

**1. Defining the Product Class**  

The `MakeupProduct` class represents an individual product in the inventory. Each product has a name, a price per unit, and a quantity that indicates how many units are available. These attributes are initialized when creating a new product instance.  

```python
class MakeupProduct:
    def __init__(self, name, price, quantity):
        self.name = name
        self.price = price
        self.quantity = quantity
```

**2. Selling a Product**

The `sell` method allows the sale of a product by reducing the stock. The amount sold is randomly chosen between one and the available stock to simulate real-world sales variation. If the stock reaches zero, the method prints a message indicating that the product is out of stock and no further sales can be made.  

```python
def sell(self):
    if self.quantity > 0:
        amount = random.randint(1, self.quantity)  
        self.quantity -= amount
        print(f"Sold {amount} units of {self.name}. Remaining stock: {self.quantity}")
    else:
        print(f"{self.name} is out of stock.")
```

**3. Displaying Product Information** 

The `displayInfo` method provides an overview of a product's details, including its name, price, and stock level. It also determines whether the product is currently available or out of stock.  

```python
def displayInfo(self):
    status = "Available" if self.quantity > 0 else "Out of stock"
    print(f"Product: {self.name}, Price: ${self.price:.2f}, Stock: {self.quantity} ({status})")
```

**4. Calculating Inventory Value**

The `inventoryValue` method calculates the total value of the product's stock using the formula: Total Value = Price * Stock Quantity

This helps determine the overall worth of the inventory at any given moment.  

```python
def inventoryValue(self):
    return self.price * self.quantity
```

**5. Example Usage**  

To demonstrate the system, three makeup products are created with predefined prices and stock levels. These products represent different types of items available in the store.  

```python
foundation = MakeupProduct("Liquid Foundation", 25.99, 15)
lipstick = MakeupProduct("Red Lipstick", 9.99, 30)
eyeshadow = MakeupProduct("Eyeshadow Palette", 35.50, 10)
```

**6. Running the Simulation**

The program first displays the initial inventory, then processes sales for each product, and finally updates the inventory to reflect the changes. The total value of the inventory is calculated after the sales have been made.  

```python
print("\nInitial Inventory:")
foundation.displayInfo()
lipstick.displayInfo()
eyeshadow.displayInfo()

print("\nSelling Products...")
foundation.sell()
lipstick.sell()
eyeshadow.sell()

print("\nUpdated Inventory:")
foundation.displayInfo()
lipstick.displayInfo()
eyeshadow.displayInfo()

totalValue = foundation.inventoryValue() + lipstick.inventoryValue() + eyeshadow.inventoryValue()
print(f"\nTotal inventory value: ${totalValue:.2f}")
```

