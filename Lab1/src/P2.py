"""
Program to calculate the payment based on hours worked and hourly rate.

This program asks the user to input the number of hours worked and the hourly rate, 
then calculates and displays the payment the user should receive.

Author: Nydia Hernández Bravo
"""

# Ask the user for the number of hours worked and the hourly rate
hoursWorked = float(input("Enter the number of hours worked: "))
hourlyRate = float(input("Enter the hourly rate: "))

# Calculate the payment
payment = hoursWorked * hourlyRate

# Display the result
print(f"The payment for {hoursWorked} hours worked at a rate of {hourlyRate} per hour is: {payment}")
