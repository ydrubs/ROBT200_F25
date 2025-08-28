#------Concept 1: Docstrings and Comments -------
"""
    - Docstrings are declared with triple quotation marks
    - Docstrings describe what a script or function does
    - Use them at the beginning of a file or inside a function
    - Use comments (#) to explain code or disable lines during testing
"""

"""
Docstring comments appear in a different shade when they are not part of the first line
"""
# A single-line comment example
a = 3  # This is an inline comment



#------Concept 2: Input / Output -------
"""
    - Use print() to output text to the terminal
    - Use input() to get user input — input() always returns a string
    - Use int(), float(), or str() to convert input types
    - Use f-strings for clean output formatting
"""

# name = input("Enter your name: ")
# age = input("How old are you? ")
#
# print(f"Hi {name}, you are {age} years old")
# print("In 5 years you will be", int(age) + 5) # Comma seperated argument (2)
# print("In 5 years you will be" + " " + str(int(age)+5)) # Treats it as one full string




#------Concept 3: Variable Assignment -------
"""
    - Use = to assign a value to a variable
    - Use compound operators (e.g., +=, *=) to update values
    - Variable names should be descriptive
    - Python variables can change type, but use consistently
"""

# x = 5
# x += 2  # x is now 7 (same x = x + 2)
# name = "R2-D2" # Python knows his is a string
# status = True # This is a boolean
#
# print(f"{name} status:", status, sep = '\n')

#------Concept 4: Operators -------
"""
    - Arithmetic: +, -, *, /, %, //, **
    - Comparison: ==, !=, <, >, <=, >=
    - Logical: and, or, not
    - Assignment: =, +=, -=, *=, etc.
"""

# Arithmetic
# a = 10 + 5 # addition
# b = 20 % 7 # modulus (remainder)
# # print(b)
# c = 2 ** 3 # Exponents
# print(c)

# # Comparison
# print(c == 8)     # True
# print(c != 10)     # True
# print(f"{c is not 10}")

# Logical
# temperature = 25
# print(temperature > 20 and temperature < 30)  # True
# print(temperature > 20 and not temperature > 30) # True




#------Concept 5: Data Types -------
"""
    - int, float, str, bool are basic types
    - list and tuple store multiple values
    - Use type() to check a variable’s type
"""

# a = 42          # int
# print(type(a))
# b = 3.14        # float
# c = "robot"     # str
# c1 = '3' # Still a string
# c2 = int(c1) # Now it's a int
# print(type(c2))
# d = True        # bool


# List (mutable)
# sensors = [20,40,50]
# sensors.append(100)
#
# print(sensors)
#
# sensors.insert(0, 15)
# print(sensors)
#
# print(sensors[1])
# sensors[1] = 30
#
# print(sensors)


# Tuple (immutable)
# position = ()
# position = (5,10)
# position.append(5) # Can't add elements to a tuple
# print(position[1])



#------Concept 6: Conditionals -------
"""
    - Use if, elif, and else to make decisions
    - Conditions must evaluate to True or False
    - Use and, or, not to combine conditions
"""

# temp = 22
# if temp > 30:
#     print("It's hot")
#
# elif temp >= 20:
#     print("Its warm")
# else:
#     print("Its cold")

temp = 32
# if temp > 30:
#     print("It's hot")
#
# if temp >= 20:
#     print("Its warm")
#
# else:
#     print("Its cold")

# battery = 80
# if battery > 50 and temp < 35:
#     print("System is ready")



#------Concept 7: Loops -------
"""
    - Use for loops to iterate through lists or ranges
    - Use while loops to repeat while a condition is true
    - Use break to exit, continue to skip one iteration
"""

# for loop with range
# for i in range(3):
#     print("Hello", i)


# for loop over list
# colors = ['red', 'green', 'blue']
# for color in colors:
#     print(f"The color is: {color}")
#
# for i in range(len(colors)):
#     print(f"The color is: {colors[i]}")



# while loop
# x = 0
# while x < 11:
#     print(f"x is: {x}")
#     x += 1
#
# y = 0
# while True:
#     print(y)
#     y += 1
#     if y > 11:
#         break



#------Concept 8: Functions -------
"""
    - Use def to define a function
    - Functions can take parameters and return values
    - Use return to send back a result
"""
# #No parameters passed
# def greet():
#     print("Hello, Human")
#
# greet()


# #One (or more parameters passed)
# def greet(name):
#     print(f"Hello {name}")
#
# a_name = input("WHat is your name: ")
# greet(a_name)


#Using keyword parameters
# def system_check(started = False, fault = False):
#     print(started, fault)
#
# system_check(fault=True, started = True)

#Using a return
# def square(n):
#     return n * n

# print(square(5))
# result = square(5)

# result += 5
# print(result)


# # Using type hints to indicate what data type Python should expect
# def data (temp : float, dist : int):
#     print('temperature:', temp)
#     print('distance', dist)
#
# data(32, 50)



#------Concept 9: Imports and Modules -------
"""
    - Use import to access standard or custom modules
    - You can import an entire module, specific functions, or use aliases
    - In ROS2, packages often import from subfolders using dot notation
"""

# # Import full module with all built-in function
import math
result = math.sqrt(16) # call the function from the module using dot-notation
print(result)


# Import specific function only
from random import randint
print(randint(1,10))



# # Import with alias, useful with modules that have long or awkward names
import time as t
for i in range(10):
    print("It's time")
    t.sleep(1)



# Import multiple functions from a module
from random import randint, random
print(randint(1,10), random())