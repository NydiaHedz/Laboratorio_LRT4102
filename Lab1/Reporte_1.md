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
