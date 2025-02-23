"""
Program to calculate the sum of the first 'n' positive integers.

This program prompts the user to enter a positive integer 'n' and calculates the sum of all integers from 1 to 'n' using the mathematical formula:

    sum = n(n + 1) / 2

Author: Nydia Hernández Bravo
"""

# Ask the user for a positive integer
n = int(input("Enter an integer: "))

# Check if the number is positive
if n > 0:
    # Calculate the sum using the mathematical formula
    sum_n = (n * (n + 1)) / 2  
    print(f"The sum of the first {n} integers is: {sum_n}")
else:
    print("Please enter a positive integer.")
