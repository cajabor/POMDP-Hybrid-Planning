# Written by Jolie Elliott
# For the Create3 iRobot

# Purpose: To follow the wall smoothly, avoid collisions, and identify door frames using a belief network.
# It records data to a CSV file and allows pausing and resuming with the 'r' key.

import asyncio
import csv
import time
import threading
from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, Robot, Create3
from irobot_edu_sdk.music import Note
from pynput import keyboard  # Make sure to install pynput for keyboard listening

# Establish connection with a specific robot
robot = Create3(Bluetooth('iRobot-030F9BF3B40449DC94031C'))
speed = 10
maxSpeed = 15
minSpeed = 1

# PID parameters
kp = 0.4
ki = 0.02
kd = 0.1

# Desired distance to the wall and threshold for front obstacles
wallDistance = 5
collisionDistance = 500
previousError = 0
integral = 0
dt = 0.1  # Time delta for PID calculation

# Initialize global variables
speedL = speed
speedR = speed
is_paused = False

# Simplified States
state = 'Wall Following'  # Possible states: 'Wall Following', 'In Door Frame', 'Exiting Door Frame', 'Passed Door Frame'

# Variables to track exiting door frame
exiting_start_position = None
exiting_traveled_distance = 0
required_exiting_distance = 30  # cm to consider door frame passed

# CSV file setup
csv_file = open('robot_data.csv', 'w', newline='')
csv_writer = csv.writer(csv_file)
csv_writer.writerow(['Time', 'State', 'SpeedL', 'SpeedR', 'd1', 'd2', 'Correction'])

# Movement Functions
async def forward():
    global speedL, speedR
    await robot.set_wheel_speeds(speedL, speedR)

async def backoff():
    await robot.set_lights_on_rgb(255, 80, 0)  # Orange for backoff
    await robot.move(-10)        # Reverse 10 cm
    await robot.turn_left(45)    # Turn left 45 degrees

async def path_out():
    """Path out of the door frame by turning right 90 degrees."""
    await robot.set_lights_on_rgb(0, 0, 255)  # Blue during path out
    await robot.turn_right(90)  # Adjust the angle as needed
    print("Executing path out of the door frame.")

def front_obstacle(sensors):
    """Check if there's an obstacle in front."""
    return sensors[3] > collisionDistance

def current_distance(d1, d2):
    """Calculate current distance to the wall using averaged sensor data."""
    ir = (d1 + d2) / 2  # Average the readings from d1 and d2 for stability
    A = 2000
    B = -150
    d = (ir - A) / B
    return d

def pid_distance(d1, d2):
    """Calculate the correction needed using PID control."""
    global integral, previousError
    d = current_distance(d1, d2)
    error = d - wallDistance
    integral += error * dt
    derivative = (error - previousError) / dt
    correction = kp * error + ki * integral + kd * derivative

    # Cap correction to prevent drastic adjustments
    correction = max(min(correction, 5), -5)
    previousError = error
    return correction

def belief_network(correction):
    """Simplified belief network to determine the robot's state."""
    global state

    if state == 'Wall Following':
        if correction > 4.0 and wallDistance > 10:
            state = 'In Door Frame'
            print("State changed to: In Door Frame")
            set_lights()
    # No transition based on correction in other states

def set_lights():
    """Set robot lights based on current state."""
    async def _set_lights():
        if state == 'Wall Following':
            await robot.set_lights_on_rgb(255, 165, 0)  # Orange
        elif state == 'In Door Frame':
            await robot.set_lights_on_rgb(255, 0, 255)  # Magenta
        elif state == 'Exiting Door Frame':
            await robot.set_lights_on_rgb(0, 0, 255)    # Blue
        elif state == 'Realigning to Wall':
            await robot.set_lights_on_rgb(128, 0, 128)  # Purple for realignment
        elif state == 'Wall Following Post-Exit':
            await robot.set_lights_on_rgb(255, 255, 0)  # Yellow for resuming wall-following
    asyncio.create_task(_set_lights())

def write_data(timestamp, d1, d2, correction):
    """Write data to the CSV file."""
    csv_writer.writerow([timestamp, state, speedL, speedR, d1, d2, correction])

def on_press(key):
    """Handle key press events."""
    global is_paused
    try:
        if key.char == 'r':
            is_paused = not is_paused
            if is_paused:
                print('Robot paused. Press "r" to resume.')
            else:
                print('Robot resumed.')
    except AttributeError:
        pass  # Ignore special keys

def start_key_listener():
    """Start the keyboard listener in a separate thread."""
    listener = keyboard.Listener(on_press=on_press)
    listener.start()

def calculate_distance(last_pos, current_pos):
    """Calculate the Euclidean distance between two positions."""
    dx = current_pos['x'] - last_pos['x']
    dy = current_pos['y'] - last_pos['y']
    distance = (dx**2 + dy**2) ** 0.5
    return distance

@event(robot.when_bumped, [True, True])
async def avoidcollision(robot_event):
    """Handle collision by backing off and transitioning states appropriately."""
    global state, exiting_start_position, exiting_traveled_distance

    if state == 'In Door Frame':
        print('Collision detected while in Door Frame! Executing backoff and marking as Exiting Door Frame.')
        await backoff()
        await path_out()
        state = 'Realigning to Wall'
        set_lights()
    else:
        print('Collision detected! Executing backoff.')
        await backoff()
        await forward()

@event(robot.when_play)
async def play(robot_event):
    """Main function to control the robot's behavior."""
    print('Please place robot near a wall.')
    await robot.set_lights_on_rgb(255, 0, 255)  # Initial Magenta
    await asyncio.sleep(1)
    await robot.set_lights_on_rgb(255, 255, 0)  # Yellow
    await asyncio.sleep(1)
    await robot.play_note(Note.C4, 1)
    await robot.set_lights_on_rgb(0, 255, 0)    # Green
    print('Starting smooth wall-following.')

    global speedL, speedR, is_paused, state, exiting_start_position, exiting_traveled_distance
    speedL = speed
    speedR = speed

    await forward()
    
    # Start the keyboard listener
    threading.Thread(target=start_key_listener, daemon=True).start()

    last_position = await get_robot_position()  # Implement this based on your SDK
    while True:
        await asyncio.sleep(0.05)  # Main loop delay

        if is_paused:
            await robot.set_wheel_speeds(0, 0)
            await robot.set_lights_on_rgb(255, 255, 255)  # White light when paused
            continue  # Skip the rest of the loop if paused

        sensors = (await robot.get_ir_proximity()).sensors

        if front_obstacle(sensors):  # Avoid front obstacle
            print('Front obstacle detected.')
            await backoff()
            await forward()
            continue  # Skip to next iteration after backoff

        # Get sensor readings
        d1 = sensors[6]
        await asyncio.sleep(0.05)
        sensors = (await robot.get_ir_proximity()).sensors
        d2 = sensors[6]
        timestamp = time.time()
        print(f'[{timestamp}] (d1, d2): ({d1}, {d2})')

        # Calculate correction with PID
        correction = pid_distance(d1, d2)
        
        # Update state based on correction
        belief_network(correction)  # Update state based on correction
        
        print(f"State: {state}, SpeedL: {speedL}, SpeedR: {speedR}, Correction: {correction}")

        # Write data to CSV
        write_data(timestamp, d1, d2, correction)

         # State transitions based on correction
        if state == 'Wall Following' and correction > 4.0:
            state = 'In Door Frame'
            set_lights()
            print("State changed to: In Door Frame")
        
        elif state == 'Realigning to Wall':
            # Rotate slowly right to find wall again
            await robot.turn_right(10)  # Turn right by 10 degrees incrementally
            await asyncio.sleep(0.2)
            
            # Check if the robot has found the wall within desired distance
            if current_distance(d1, d2) <= wallDistance + 2:
                state = 'Wall Following Post-Exit'
                set_lights()
                print("State changed to: Wall Following Post-Exit")

        elif state == 'Wall Following Post-Exit':
            # Continue with normal wall following behavior
            speedL = max(min(speed + correction, maxSpeed), minSpeed)
            speedR = max(min(speed - correction, maxSpeed), minSpeed)
            await forward()
            print("Resuming normal wall-following behavior.")

        # Write data to CSV
        write_data(timestamp, d1, d2, correction)

        # Move forward based on PID correction
        if state == 'Wall Following' or state == 'In Door Frame':
            speedL = max(min(speed + correction, maxSpeed), minSpeed)
            speedR = max(min(speed - correction, maxSpeed), minSpeed)
            await forward()

async def get_robot_position():
    """
    Placeholder function to get the robot's current position.
    Implement this based on your SDK's capabilities.
    """
    # Example implementation (replace with actual position retrieval)
    # Assuming the SDK provides a method to get the robot's position
    # Here, we'll return a dummy position that increments over time
    # Replace this with actual position data from the robot
    # For the purpose of this example, let's assume the robot moves straight
    # and we can estimate the position based on time and speed

    # Initialize position on first call
    if not hasattr(get_robot_position, "position"):
        get_robot_position.position = {"x": 0, "y": 0, "orientation": 0}

    # Update position based on current wheel speeds and time
    # This is a simplistic estimation and should be replaced with actual sensor data
    get_robot_position.position['x'] += speed * dt * 0.1  # Assuming speed is cm/s
    # y and orientation can be updated similarly if needed
    return get_robot_position.position

# Close the CSV file gracefully when the program exits
def close_csv_file():
    csv_file.close()

# Register the CSV file close function to be called on exit
import atexit
atexit.register(close_csv_file)

robot.play()
