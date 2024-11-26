# Written by Jolie Elliott
# For the Create3 Irobot

import asyncio
import socket
import keyboard
import subprocess
from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, Robot, Create3
from irobot_edu_sdk.music import Note
import numpy as np
from scipy.optimize import curve_fit
import math



#from belief_network_jollie import update_belief_state  # Import the belief network function
from belief_network import update_belief_state  # Import the belief network function


# Establish connection with a specific robot
robot = Create3(Bluetooth('iRobot-88FA7A7E3FCC461E8B675C'))  # Robot 2

# Set up the socket connection for UI communication
HOST = '127.0.0.1'  # Localhost for testing; replace with actual IP if needed
PORT = 65432  # Port number for UI to listen to

# Start Trace_ui Process
trace_ui_process = subprocess.Popen(['python', 'C:/Users/scuba/Documents/CSE_5694/CSE5694_Project1/irobot_env/Scripts/Project3_withTrace/trace_ui.py'])

# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# PID parameters and other settings
kp = 0.4
ki = 0.02
kd = 0.1
wallDistance = 6
collisionDistance = 500
minTurnDistance = 7
sharpTurnDistance = 4
maxSpeed = 21
minSpeed = 1
forwardSpeed = 5
TURN_SPEED = 5
previousError = 0.1
integral = 0.1
dt = 0.1
speedL = forwardSpeed
speedR = forwardSpeed
run_time = 0

# Global flag to control wall-following and returning behavior
returning_to_start = False

# Initialize variables for starting position
starting_x, starting_y = None, None

# Smoothing settings
heading_window_size = 5  # Number of recent headings to smooth over
distance_filter_threshold = 0.5  # Min difference to accept distance updates
previous_positions = []
recent_headings = []
last_wall_position = None

# Track current LED color
current_led_color = ""

# Additional settings for wall smoothing
wall_smoothing_window = 5  # Number of recent wall positions to smooth over
wall_positions = []  # Store recent wall positions for averaging


# Model function for converting sensor readings to distance
def model_function(reading, A, B):
    return A + B * np.log(reading)

# Calibration data for IR sensors
readings = np.array([2525, 1560, 1060, 660, 520, 350, 270, 200, 150, 120, 90, 70, 50, 40, 30, 24, 15])
distances = np.array([1, 1.5, 2, 2.5, 3, 3.5, 4, 4.5, 5, 5.5, 6, 6.5, 7, 7.5, 8, 8.5, 9])
params, _ = curve_fit(model_function, readings, distances)
A, B = params


# =========================================================================
# Main Function Classes
# =========================================================================

# Function to convert sensor reading to distance
def approximate_distance(sensor_reading):
    if sensor_reading > 0:
        distance = A + B * np.log(sensor_reading)
        if np.isinf(distance) or np.isnan(distance):
            return 999
        return round(distance, 2)
    else:
        return 999

# Function to send position and wall data to UI
def send_position_to_ui(x, y, wall_x, wall_y):
    """Send the (x, y) position and wall position (wall_x, wall_y) to the UI over UDP."""
    message = f"{x},{y},{wall_x},{wall_y}"
    sock.sendto(message.encode(), (HOST, PORT))
   # print(f"Sent position to UI: {message}")

# PID control for maintaining distance to the wall
def pid_distance(d1, d2):
    global integral, previousError
    d = (d1 + d2) / 2
    if np.isinf(d) or np.isnan(d):
        d = wallDistance
    error = d - wallDistance
    integral += error * dt
    derivative = (error - previousError) / dt
    correction = kp * error + ki * integral + kd * derivative
    correction = max(min(correction, 5), -5)
    if np.isnan(correction):
        correction = 0
    previousError = error
    return correction

# Helper to calculate smoothed heading
def calculate_smoothed_heading():
    if len(recent_headings) < 2:
        return None
    return sum(recent_headings) / len(recent_headings)

# Movement Functions
async def forward():
    global speedL, speedR
    speedL = max(min(speedL, maxSpeed), minSpeed)
    speedR = max(min(speedR, maxSpeed), minSpeed)
    # Forward function will only update color if no belief state color change was triggered
    if current_led_color is None: 
        await robot.set_lights_on_rgb(0, 255, 0)
    await robot.set_wheel_speeds(speedL, speedR)

async def backoff_left():
    await robot.set_lights_on_rgb(255, 80, 0)
    await robot.move(-10)
    await robot.turn_left(20)

async def backoff_right():
    await robot.set_lights_on_rgb(255, 80, 0)
    await robot.move(-10)
    await robot.turn_left(20)

def front_obstacle(sensors):
    front_distance = min(approximate_distance(sensors[3]), approximate_distance(sensors[4]))
    return front_distance

@event(robot.when_bumped, [False, True])
async def avoidcollision(robot):
    print('Right Bumper Collision detected! Executing backoff.')
    await backoff_right()
    await forward()

@event(robot.when_bumped, [True, True])
async def avoidcollision(robot):
    print('Left Bumper Collision detected! Executing backoff.')
    await backoff_left()
    await forward()

def smoothed_wall_position(x, y, d_wall, smoothed_heading):
    """
    Calculate and smooth the wall position based on recent wall positions.
    """
    # Calculate the raw wall position based on distance and heading
    sensor_angle = math.radians(25)  # Sensor angle offset to the right
    raw_wall_x = x + d_wall * math.cos(smoothed_heading - sensor_angle)
    raw_wall_y = y + d_wall * math.sin(smoothed_heading - sensor_angle)

    # Add to wall positions and maintain a rolling window
    wall_positions.append((raw_wall_x, raw_wall_y))
    if len(wall_positions) > wall_smoothing_window:
        wall_positions.pop(0)

    # Calculate the average of recent wall positions
    avg_wall_x = sum(pos[0] for pos in wall_positions) / len(wall_positions)
    avg_wall_y = sum(pos[1] for pos in wall_positions) / len(wall_positions)

    return avg_wall_x, avg_wall_y


# Function to listen for keyboard events asynchronously
async def keyboard_listener():
    global returning_to_start

    while True:
        if keyboard.is_pressed("c"):
            await stop_robot()
            print("Stop key 'C' pressed. Robot stopped.")
            await asyncio.sleep(0.2)  # Debounce delay for key press

        elif keyboard.is_pressed("r"):
            await return_to_start()
            print("Return key 'R' pressed. Returning to start position.")
            await asyncio.sleep(0.2)  # Debounce delay for key press

        await asyncio.sleep(0.1)  # Small delay to prevent high CPU usage

# Function to return to the starting position using navigate_to
async def return_to_start():
    global starting_x, starting_y, returning_to_start

    if starting_x is None or starting_y is None:
        print("Starting position not set.")
        return

    # Set the flag to indicate returning to start
    returning_to_start = True

    # Set the LED to indicate returning to start
    color = await update_belief_state(
        starting_x, starting_y, [], 
        stop_robot=stop_robot, 
        return_to_start=return_to_start, 
        door_count_goal=3,  # Set your desired door count goal
        override_state="return_to_home"
    )
    if color:
        await set_led_color(color)

    # Navigate to the starting position
    await robot.navigate_to(starting_x-50, starting_y)
    

    # Continuously check if the robot has reached the start location
    # while returning_to_start:
    #     position = await robot.get_position()
    #     if position:
    #         # Check if the robot is within a very close threshold of the starting point
    #         if abs(position.x - starting_x) <= 0.1 and abs(position.y - starting_y) <= 0.1:
    #             await stop_robot()  # Stop the robot when it reaches the start
    #             await set_led_color("red")  # Set LED to red to indicate it has returned home
    #             print("Robot returned to starting position and stopped with red LED.")
    #             returning_to_start = False  # Reset the return flag
    #             break
    #     await asyncio.sleep(0.1)  # Check position periodically


    #     # Call update_belief_state with dummy positions to handle state and LED updates if needed
    #     await update_belief_state(
    #         robot_x=starting_x, 
    #         robot_y=starting_y, 
    #         sensor_distances=[], 
    #         stop_robot=stop_robot, 
    #         return_to_start=return_to_start,
    #         last_position=(starting_x, starting_y), 
    #         current_position=(position.x, position.y) if position else (starting_x, starting_y)
    #     )
    #await robot.turn_left(180)

# Function to stop the robot in place
async def stop_robot():
    """Stop the robot in place."""
    global returning_to_start
    returning_to_start = False  # Stop the return mode if it was active
    await robot.set_wheel_speeds(0, 0)  # Stop the robot wheels
    await set_led_color("red")  # Set LED to red to indicate stopping
    print("Robot stopped.")

# Define a function to set the LED color
async def set_led_color(color):
    global current_led_color
    if color != current_led_color:
        current_led_color = color
        if color == "blue":
            await robot.set_lights_on_rgb(0, 0, 255)
        elif color == "magenta":
            await robot.set_lights_on_rgb(255, 0, 255)
        elif color == "yellow":
            await robot.set_lights_on_rgb(255, 255, 0)
        elif color == "green":
            await robot.set_lights_on_rgb(0, 255, 0)
        elif color == "red":
            await robot.set_lights_on_rgb(255, 0, 0)
        print(f"LED color updated to {color}")



# =========================================================================
# Main Class which runs the Robot
# =========================================================================
@event(robot.when_play)
async def play(robot):
    print('Starting smooth wall-following with PID control and obstacle avoidance.')
    global speedL, speedR, last_wall_position, current_led_color, returning_to_start, run_time
    speedL = forwardSpeed
    speedR = forwardSpeed
    await forward()

    global starting_x, starting_y
    position = await robot.get_position()
    if position:
        starting_x, starting_y = position.x, position.y
        print(f"Starting position set to: ({starting_x}, {starting_y})")

    # Initialize observation sequence as an empty list
    observation_sequence = []

    # Proximity distance to check near starting point, only active when returning home
    proximity_threshold = 15

    # Set the number of doors the robot should pass before returning home
    asyncio.create_task(keyboard_listener())

    try:
        while True:
            run_time += 0.2;
            # Skip wall-following if in return mode
            if returning_to_start:
                await asyncio.sleep(0.1)
                continue

            # Retrieve IR sensor distances
            sensors = (await robot.get_ir_proximity()).sensors
            distances = [approximate_distance(sensor) for sensor in sensors]

            # Use only IR sensor 6 for detecting close or far
            current_observation = distances[6]
            observation_sequence.append(current_observation)

            position = await robot.get_position()
            if position:
                x, y = position.x, position.y
                previous_positions.append((x, y))
                if len(previous_positions) > 2:
                    previous_positions.pop(0)

                if len(previous_positions) == 2:
                    x1, y1 = previous_positions[0]
                    x2, y2 = previous_positions[1]
                    heading = math.atan2(y2 - y1, x2 - x1)
                    recent_headings.append(heading)
                    if len(recent_headings) > heading_window_size:
                        recent_headings.pop(0)

                    smoothed_heading = calculate_smoothed_heading()
                    if smoothed_heading is not None:
                        d_wall = distances[6]
                        wall_x, wall_y = smoothed_wall_position(x, y, d_wall, smoothed_heading)
                        if last_wall_position is None or (
                            abs(last_wall_position[0] - wall_x) > distance_filter_threshold or 
                            abs(last_wall_position[1] - wall_y) > distance_filter_threshold
                        ):
                            send_position_to_ui(x, y, wall_x, wall_y)
                            last_wall_position = (wall_x, wall_y)
            else:
                print("Position data not available.")

            # Check if within proximity to starting point when returning to start
            if returning_to_start and position:
                distance_to_start = math.sqrt((position.x - starting_x)**2 + (position.y - starting_y)**2)
                if distance_to_start <= proximity_threshold:
                    print("Within proximity to starting point while returning home. Stopping robot.")
                    await stop_robot()
                    break

            # Wall-following PID control, active only when not returning to start
            if not returning_to_start:
                front_distance = front_obstacle(sensors)
                if front_distance < minTurnDistance:
                    sharpness = max(1, (sharpTurnDistance / front_distance) * TURN_SPEED)
                    await robot.set_wheel_speeds(-sharpness, sharpness)
                    continue

                d1 = distances[6]
                
                await asyncio.sleep(0.05)
                sensors = (await robot.get_ir_proximity()).sensors
                d2 = approximate_distance(sensors[6])
                
                # Check the difference between d2 and d1 to detect gaps or doors
                # if d2 - d1 > 11:  # Likely approaching a door or gap in the wall
                #     print("Gap detected, moving forward without PID correction.")
                #     # Move forward without correction
                #     # await robot.navigate_to(0, y + 12)
                #     speedL = forwardSpeed
                #     speedR = forwardSpeed
                # else:
                # Wall is still present, use PID correction
                correction = pid_distance(d1, d2)
                speedL = max(min(forwardSpeed + correction, maxSpeed), minSpeed)
                speedR = max(min(forwardSpeed - correction, maxSpeed), minSpeed)
                if run_time >= 1.5:  # Wait for PID to correct orientation before updating belief
                    # Pass observation_sequence to update_belief_state
                    new_color = await update_belief_state(
                        x, y, distances,
                        stop_robot=stop_robot,
                        return_to_start=return_to_start,
                        door_count_goal= 3 

                    )


                    # Check if the robot should stop completely after returning home
                    if new_color == "off":
                        print("Robot has completed all tasks and will now stop completely.")
                        break  # Exit the loop to stop further execution

                    # Set the LED color based on the belief state
                    if new_color and new_color != current_led_color:
                        await set_led_color(new_color)

            await forward()
            await asyncio.sleep(0.1)

    finally:
        sock.close()

robot.play()
