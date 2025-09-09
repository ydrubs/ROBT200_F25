"""
This program drives the 3pi robot in a straight line using PID control.

Each component (Proportional, Integral, Derivative) is calculated in its own function
to make the control algorithm easier to understand and test.
"""

from pololu_3pi_2040_robot import robot
import time

# === [1] Initialize system ===

# Hardware setup
encoders = robot.Encoders()
button_a = robot.ButtonA()
motors = robot.Motors()
display = robot.Display()

# === Student-tunable parameters ===
target_value = 20000    # Distance to travel in encoder ticks
base_speed = motors.MAX_SPEED // 2    # Base motor speed (3000)

Kp = 30   # Proportional gain
Ki = 1    # Integral gain
Kd = 0    # Derivative gain

# === Internal PID variables ===
total_error = 0 # Store the total error
last_error = 0  # Store the error from the previous cycle
last_time = time.ticks_ms() # store the time it took to run the previous cycle

run = False # Flag to initialize the system

# === [4] PID component functions ===

def calculate_proportional(error):
    """
    Calculates the proportional term.
    This term responds directly to how far off the robot is right now.
    A bigger error creates a stronger correction.
    """
    return Kp * error# Takes the error and adjusts it by some correction factor defined by kP

def calculate_integral(error, dt):
    """
    Calculates the integral term.
    This term adds up small errors over time to correct long-term drift.
    It helps when the robot consistently veers one way even with small immediate errors.
    """
    global total_error # We need to update this variable globally since it represents total system error
    total_error += error * dt # add the amount of error that has accumulated over the course of a timeframe (dt) to the current error

    # Clamp to prevent integral windup (over-correction)
    total_error = max(-1000, min(1000, total_error)) # the 1000 acts as a limit for the error preventing integral windup (6000 is max_speed)
    return Ki * total_error # return an integral correction value

def calculate_derivative(error, dt):
    """
    Calculates the derivative term.
    This term looks at how quickly the error is changing and helps slow down sudden corrections.
    It helps prevent overshooting and makes the response smoother.
    """
    global last_error # This needs to be updated globally since we need to keep track of the previous error

    # elapsed time is zero or negative (due to skipping or glitch) make derivative zero (or we'll get an ERROR when dividing)
    # Otherwise we compute a derivative term that is the average of the time interval
    if dt > 0:
        derivative = (error - last_error) / dt
    else:
        derivative = 0

    last_error = error # save the error as last error for the next clock cycle
    return  Kd * derivative # return the derivative value

def compute_pid_correction(error, dt):
    """
    Combines the proportional, integral, and derivative terms to calculate
    the total correction. This final value will be used to adjust the motor speeds.
    """
    P = calculate_proportional(error) # Call FUnction to calculate P-value (Kp)
    I = calculate_integral(error, dt) # Call Function to calculate I-value (Ki)
    D = calculate_derivative(error, dt) # Call Function to calculate D-value (Kd)
    return P + I + D # Return the combine correction to send to motors

# === Main Loop ===
while True:

    # === [2] Track time (dt) ===
    current_time = time.ticks_ms() # timme of current cycle
    dt = time.ticks_diff(current_time, last_time) / 1000  # Convert to seconds
    last_time = current_time # reset set last cycle as current time to compare

    # === [3] Read encoders and compute error ===
    left_count, right_count = encoders.get_counts() # get encoder values on both wheels and save to two variables
    error = left_count - right_count # Store the error (diff between the two readings)

    # === [4] and [5] Compute PID correction ===
    # call the function that handles all the PID calculations
    correction = compute_pid_correction(error, dt)  # We pass in the error and the time differene between two loops

    # === [6] Adjust motor speeds based on correction ===
    left_pwm = base_speed - correction # float values
    right_pwm = base_speed + correction

    # Clamp motor values
    """
    max() takes the higher of either -base_speed or the result of min(base_speed, motor speed), 
    ensuring the final value is between -base_speed and +base_speed.
    """
    left_pwm = max(-base_speed, min(base_speed, int(left_pwm))) # max() ensures the value doesn't go below -base_speed
    right_pwm = max(-base_speed, min(base_speed, int(right_pwm))) #

    # === Display debug info ===
    display.fill_rect(0, 0, 128, 40, 0)
    display.text("L:" + str(left_count), 0, 0)
    display.text("R:" + str(right_count), 64, 0)
    display.text("Err:" + str(int(error)), 0, 10)
    display.text("Int:" + str(int(total_error)), 0, 20)
    display.text("Der:" + str(int(last_error)), 64, 20)
    display.show()

    # === [7] Start/Stop with button A ===
    if button_a.check() and button_a.is_pressed(): # Checks for a press AND release before executing

        if not run: # Starts motion
            time.sleep(0.5)
            run = True
            encoders.get_counts(reset=True)
            total_error = 0
            last_error = 0
            last_time = time.ticks_ms()

        else:  # Stops motion
            run = False

    # === [8] Stop when target reached ===

    if run and (left_count >= target_value or right_count >= target_value): # Check if either encoder rached target value
        run = False
        motors.set_speeds(0, 0)
        display.fill_rect(0, 0, 128, 30, 0)
        display.text("Target reached!", 0, 0)
        display.show()
        continue

    # === [9] Drive motors ===
    if run:
        motors.set_speeds(left_pwm, right_pwm)
    else:
        motors.set_speeds(0,0)

    time.sleep_ms(50) # Short delay to allow code to resolve on the robot
