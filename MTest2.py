import asyncio
import csv
import time
import threading
from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, Robot, Create3
from irobot_edu_sdk.music import Note
from pynput import keyboard

# Robot initialization
robot = Create3(Bluetooth('iRobot-030F9BF3B40449DC94031C'))
speed = 5
maxSpeed = 15
minSpeed = 1

# PID parameters
kp = 0.4
ki = 0.02
kd = 0.1
wallDistance = 8.5
collisionDistance = 500
previousError = 0
integral = 0
dt = 0.1

# Global variables
speedL = speed
speedR = speed
is_paused = False
bumper_triggered = False
# States: 'Wall Following', 'In Door Frame', 'Exiting Door Frame', 'Realigning to Wall', 'Wall Following Post-Exit'
state = 'Wall Following'
door_frame_start_time = None
last_door_detection_time = None

# Enhanced CSV file setup with door detection
csv_file = open('robot_sensor_data.csv', 'w', newline='')
csv_writer = csv.writer(csv_file)
csv_writer.writerow(['Bumper_Triggered', 'Current_Distance', 'Door_Passed'])


async def forward():
    global speedL, speedR
    await robot.set_wheel_speeds(speedL, speedR)

async def backoff():
    await robot.set_lights_on_rgb(255, 80, 0)  # Orange for backoff
    await robot.move(-10)        
    await robot.turn_left(45)    

async def path_out():
    """Path out of the door frame"""
    await robot.set_lights_on_rgb(0, 0, 255)  
    await robot.turn_right(90)  
    print("Executing path out of door frame.")

def current_distance(d1, d2):
    """Calculate current distance to wall"""
    ir = (d1 + d2) / 2
    A = 2000
    B = -150
    d = (ir - A) / B
    return d

def pid_distance(d1, d2):
    """Calculate PID correction"""
    global integral, previousError
    d = current_distance(d1, d2)
    error = d - wallDistance
    integral += error * dt
    derivative = (error - previousError) / dt
    correction = kp * error + ki * integral + kd * derivative
    correction = max(min(correction, 5), -5)
    previousError = error
    return correction, d

def check_door_frame(d1, d2, correction):
    """Check if robot is passing through a door frame"""
    global state, door_frame_start_time, last_door_detection_time
    current_time = time.time()
    
    # Door frame detection conditions
    distance = current_distance(d1, d2)
    is_door_frame = correction > 3.0 and distance > (wallDistance + 5)
    
    if state == 'Wall Following' and is_door_frame:
        state = 'In Door Frame'
        door_frame_start_time = current_time
        last_door_detection_time = current_time
        print("Door frame detected! State changed to: In Door Frame")
        return True
    
    elif state == 'In Door Frame':
        if not is_door_frame:
            state = 'Exiting Door Frame'
            print("Exiting door frame")
            
    elif state == 'Exiting Door Frame':
        if current_time - door_frame_start_time > 2.0:  # 2 seconds timeout
            state = 'Realigning to Wall'
            print("Realigning to wall")
            
    return False

def set_lights():
    """Set robot lights based on current state"""
    async def _set_lights():
        if state == 'Wall Following':
            await robot.set_lights_on_rgb(255, 165, 0)  # Orange
        elif state == 'In Door Frame':
            await robot.set_lights_on_rgb(255, 0, 255)  # Magenta
        elif state == 'Exiting Door Frame':
            await robot.set_lights_on_rgb(0, 0, 255)    # Blue
        elif state == 'Realigning to Wall':
            await robot.set_lights_on_rgb(128, 0, 128)  # Purple
        elif state == 'Wall Following Post-Exit':
            await robot.set_lights_on_rgb(255, 255, 0)  # Yellow
    asyncio.create_task(_set_lights())

def write_sensor_data(timestamp, d1, d2, correction, curr_distance, door_detected=False):
    """Write enhanced sensor data with door detection to CSV"""
    time_since_last_door = 0
    if last_door_detection_time is not None:
        time_since_last_door = timestamp - last_door_detection_time
        
    csv_writer.writerow([
        #timestamp,
        #state,
        bumper_triggered,
        round(curr_distance, 2),
        #d1,
        #d2,
        #round(correction, 2),
        #round(speedL, 2),
        #round(speedR, 2),
        door_detected#,
        #round(time_since_last_door, 2)
    ])

@event(robot.when_bumped, [True, True])
async def handle_collision(robot_event):
    """Handle collision events"""
    global bumper_triggered, state
    bumper_triggered = True
    
    print('Collision detected!')
    await robot.set_lights_on_rgb(255, 0, 0)  # Red for collision
    
    if state == 'In Door Frame':
        print('Collision in Door Frame! Executing recovery...')
        await robot.move(-20)
        await robot.turn_left(-45)
        await robot.move(20)
        await backoff()
        await path_out()
        state = 'Realigning to Wall'
    else:
        await robot.move(-20)
        await robot.turn_left(-45)
        await robot.move(20)
        await backoff()
    
    # Record collision event
    timestamp = time.time()
    sensors = (await robot.get_ir_proximity()).sensors
    d1, d2 = sensors[6], sensors[6]
    correction, curr_distance = pid_distance(d1, d2)
    write_sensor_data(timestamp, d1, d2, correction, curr_distance)
    
    await asyncio.sleep(1)
    bumper_triggered = False
    set_lights()

@event(robot.when_play)
async def play(robot_event):
    """Main control loop with door detection"""
    global speedL, speedR, is_paused, state
    
    print('Starting wall following with door detection...')
    set_lights()
    
    while True:
        await asyncio.sleep(dt)
        
        if is_paused:
            await robot.set_wheel_speeds(0, 0)
            continue
            
        # Get sensor readings
        sensors = (await robot.get_ir_proximity()).sensors
        d1 = sensors[6]
        await asyncio.sleep(0.05)
        sensors = (await robot.get_ir_proximity()).sensors
        d2 = sensors[6]
        
        # Calculate PID correction and current distance
        correction, curr_distance = pid_distance(d1, d2)
        
        # Check for door frame
        door_detected = check_door_frame(d1, d2, correction)
        
        # State-based behavior
        if state == 'Realigning to Wall':
            await robot.turn_right(10)
            await asyncio.sleep(0.2)
            if current_distance(d1, d2) <= wallDistance + 2:
                state = 'Wall Following Post-Exit'
                set_lights()
        
        elif state in ['Wall Following', 'Wall Following Post-Exit']:
            speedL = max(min(speed + correction, maxSpeed), minSpeed)
            speedR = max(min(speed - correction, maxSpeed), minSpeed)
            await forward()
        
        # Record data
        timestamp = time.time()
        write_sensor_data(timestamp, d1, d2, correction, curr_distance, door_detected)
        
        # Print status
        print(f'Time: {timestamp:.2f}, State: {state}, Distance: {curr_distance:.2f}cm, Correction: {correction:.2f}')

def on_press(key):
    """Handle keyboard events"""
    global is_paused
    try:
        if key.char == 'r':
            is_paused = not is_paused
            print('Robot ' + ('paused' if is_paused else 'resumed'))
    except AttributeError:
        pass

def cleanup():
    """Cleanup function"""
    csv_file.close()
    print("Data collection completed. CSV file closed.")

# Setup keyboard listener and cleanup
keyboard_listener = keyboard.Listener(on_press=on_press)
keyboard_listener.start()
import atexit
atexit.register(cleanup)

# Start the robot
robot.play()

