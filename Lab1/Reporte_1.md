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

#### Problem 1: Sum of Positive Integers

Write a program that reads a positive integer "n" entered by the user and then displays  
the sum of all integers from 1 to n.

The sum of the first n positive integers can be calculated as follows:

S = n(n + 1) / 2

##### Solution

The solution implemented was:

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

## Problem 2: Salary Calculation

Write a program that asks the user for the number of hours worked and the hourly wage. Then, display the corresponding payment on the screen.

##### Solution

The solution implemented was:

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
