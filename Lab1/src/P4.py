"""
Program to calculate the average of even numbers and the product of odd numbers.

This program creates a list of at least 10 numbers, calculates the average of the even numbers, 
and computes the product of the odd numbers. Finally, it prints the results.

Author: Nydia Hernández Bravo
"""

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
