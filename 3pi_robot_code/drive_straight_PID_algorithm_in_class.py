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
pass    # Distance to travel in encoder ticks
pass    # Base motor speed

pass    # Proportional gain
pass    # Integral gain
pass    # Derivative gain

# === Internal PID variables ===
pass # Store the total error
pass # Store the error from the previous cycle
pass # store the time it took to run the previous cycle

pass # Flag to initialize the system

# === [4] PID component functions ===

def calculate_proportional(error):
    """
    Calculates the proportional term.
    This term responds directly to how far off the robot is right now.
    A bigger error creates a stronger correction.
    """
    pass # Takes the error and adjusts it by some correction factor defined by kP

def calculate_integral(error, dt):
    """
    Calculates the integral term.
    This term adds up small errors over time to correct long-term drift.
    It helps when the robot consistently veers one way even with small immediate errors.
    """
    pass # We need to update this variable globally since it represents total system error
    pass # add the amount of error that has accumulated over the course of a timeframe (dt) to the current error

    # Clamp to prevent integral windup (over-correction)
    pass # the 1000 acts as a limit for the error preventing integral windup (6000 is max_speed)
    pass # return an integral correction value

def calculate_derivative(error, dt):
    """
    Calculates the derivative term.
    This term looks at how quickly the error is changing and helps slow down sudden corrections.
    It helps prevent overshooting and makes the response smoother.
    """
    pass # This needs to be updated globally since we need to keep track of the previous error


    # elapsed time is zero or negative (due to skipping or glitch) make derivative zero (or we'll get an ERROR when dividing)
    # Otherwise we compute a derivative term that is the average of the time interval
    pass


    pass # save the error as last error for the next clock cycle

    pass # return the derivative value

def compute_pid_correction(error, dt):
    """
    Combines the proportional, integral, and derivative terms to calculate
    the total correction. This final value will be used to adjust the motor speeds.
    """
    pass # Call FUnction to calculate P-value (Kp)
    pass # Call Function to calculate I-value (Ki)
    pass # Call Function to calculate D-value (Kd)
    pass # Return the combine correction to send to motors

# === Main Loop ===
while True:

    # === [2] Track time (dt) ===
    pass # timme of current cycle
    pass  # Convert to seconds
    pass # reset set last cycle as current time to compare

    # === [3] Read encoders and compute error ===
    pass # get encoder values on both wheels and save to two variables
    pass # Store the error (diff between the two readings)

    # === [4] and [5] Compute PID correction ===
    # call the function that handles all the PID calculations
    pass  # We pass in the error and the time differene between two loops

    # === [6] Adjust motor speeds based on correction ===
    pass
    pass

    # Clamp motor values
    """
    max() takes the higher of either -base_speed or the result of min(base_speed, motor speed), 
    ensuring the final value is between -base_speed and +base_speed.
    """
    pass # max() ensures the value doesn't go below -base_speed
    pass #

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

        pass # Starts motion
            time.sleep(0.5)
            run = True
            encoders.get_counts(reset=True)
            total_error = 0
            last_error = 0
            last_time = time.ticks_ms()

        else:  # Stops motion
            run = False

    # === [8] Stop when target reached ===

    pass # Check if either encoder rached target value
        run = False
        motors.set_speeds(0, 0)
        display.fill_rect(0, 0, 128, 30, 0)
        display.text("Target reached!", 0, 0)
        display.show()
        continue

    # === [9] Drive motors ===
    pass

    time.sleep_ms(50) # Short delay to allow code to resolve on the robot
