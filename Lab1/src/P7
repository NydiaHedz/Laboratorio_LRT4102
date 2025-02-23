"""
Basic Inventory Management System

This program allows a store to manage its inventory by:
- Creating products with a name, price, and stock quantity.
- Updating the stock when products are sold.
- Displaying product information.
- Calculating the total inventory value.

Author: Nydia Hernández Bravo
"""

import random

class MakeupProduct:
    def __init__(self, name, price, quantity):
        """Initialize a makeup product with name, price, and stock quantity."""
        self.name = name
        self.price = price
        self.quantity = quantity

    def sell(self):
        """Sell a random quantity of the product, ensuring it does not exceed stock."""
        if self.quantity > 0:
            amount = random.randint(1, self.quantity)  # Random amount between 1 and available stock
            self.quantity -= amount
            print(f"Sold {amount} units of {self.name}. Remaining stock: {self.quantity}")
        else:
            print(f"{self.name} is out of stock.")

    def displayInfo(self):
        """Display product information."""
        status = "Available" if self.quantity > 0 else "Out of stock"
        print(f"Product: {self.name}, Price: ${self.price:.2f}, Stock: {self.quantity} ({status})")

    def inventoryValue(self):
        """Calculate the total value of this product in inventory."""
        return self.price * self.quantity


# Example usage
foundation = MakeupProduct("Liquid Foundation", 25.99, 15)
lipstick = MakeupProduct("Red Lipstick", 9.99, 30)
eyeshadow = MakeupProduct("Eyeshadow Palette", 35.50, 10)

# Display initial stock
print("\nInitial Inventory:")
foundation.displayInfo()
lipstick.displayInfo()
eyeshadow.displayInfo()

# Sell random amounts of products
print("\nSelling Products...")
foundation.sell()
lipstick.sell()
eyeshadow.sell()

# Display updated stock
print("\nUpdated Inventory:")
foundation.displayInfo()
lipstick.displayInfo()
eyeshadow.displayInfo()

# Calculate total inventory value
totalValue = foundation.inventoryValue() + lipstick.inventoryValue() + eyeshadow.inventoryValue()
print(f"\nTotal inventory value: ${totalValue:.2f}")
