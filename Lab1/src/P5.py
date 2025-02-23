"""
Guess the Secret Number Game

This program generates a random number between 1 and 10, and the user must try to guess it.
The program provides hints if the guessed number is too high or too low.
It keeps running until the user guesses correctly and then displays the number of attempts.

Author: Nydia Hernández Bravo
"""

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
