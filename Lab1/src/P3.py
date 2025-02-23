"""
Program to calculate and display the total payment for multiple operators.

This program stores a list of operators with their hourly wage and hours worked.
It calculates the total salary for each operator and prints their name along with the amount to be paid.

Author: Nydia Hernández Bravo
"""

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
