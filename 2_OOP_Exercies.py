"""
Programming Exercise

Objective:
    1. Create a basic Robot class with attributes and a method
    2. Create a DeliveryRobot class that inherits from Robot and adds new features
    3. Practice using super() to reuse the parent constructor

Instructions:
    Step 1: Define a Robot class
        - The constructor should take name and battery_level
        - Add a method called status() that prints the robot's name and battery level

    Step 2: Define a DeliveryRobot class that inherits from Robot
        - The constructor should take name, battery_level, and delivery_item
        - Use super() to initialize the name and battery level from the parent class
        - Add a method called deliver() that prints a message like:
                "Delivering [item]..."

    Step 3: Demonstrate usage
        - Create one regular Robot and one DeliveryRobot
        - Call their status() methods
        - Call deliver() only on the DeliveryRobot


Example Output (Expected)
    Name: Bolt, Battery: 85%
    Name: R2-D2, Battery: 60%
    Delivering package...
"""

