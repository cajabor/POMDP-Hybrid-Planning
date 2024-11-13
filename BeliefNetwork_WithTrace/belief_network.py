import math
import random
import time
import asyncio
import csv
import os
import numpy as np

# State variables
state = "wall_following"  # Initial state
previous_heading = None
door_start_heading = None
recent_positions = []
door_states = [False]       #Assuming the first state is when were not in a door
distance_travelled_in_frame = 0  # Track distance traveled since potential frame detection
distance_travelled_after_door = 0  # Track distance traveled after passing door frame
doors_passed = 0  # Count of doors passed

# Thresholds
distance_increase_threshold = 8 # Increase for possible door frame start
distance_decrease_threshold = 6  # Decrease to detect the other side of a door frame
angle_tolerance = 0.1  # Tolerance for heading change to verify original path
distance_travel_threshold = 5  # Distance threshold to confirm door frame detection
distance_after_door_threshold = 30  # Distance to travel after passing the door frame, in cm

# Function to calculate heading based on recent positions
def calculate_heading(x1, y1, x2, y2):
    return math.atan2(y2 - y1, x2 - x1)

# Main function to update the belief state and determine LED color
async def update_belief_state(robot_x, robot_y, sensor_distances, stop_robot, return_to_start, door_count_goal, observation_sequence=None, override_state=None):
    global state, previous_heading, door_start_heading, recent_positions, distance_travelled_in_frame, distance_travelled_after_door, doors_passed

    # Handle any override state, such as 'return_to_home' or 'stop'
    if override_state:
        state = override_state
        if state == "return_to_home" or state == "stop":
            print(f"Override State Activated: {state}")
            await stop_robot()  # Call stop_robot function immediately
            return "red"
        
    if state == "possible_door_frame":
        door_states.append(True)        #keep track of the times were in a door or not
    else:
        door_states.append(False)

    # Track recent positions to calculate heading
    recent_positions.append((robot_x, robot_y))
    if len(recent_positions) > 2:
        recent_positions.pop(0)  # Keep only the last two positions

    # Calculate heading if there are enough positions
    if len(recent_positions) == 2:
        x1, y1 = recent_positions[0]
        x2, y2 = recent_positions[1]
        heading = calculate_heading(x1, y1, x2, y2)

        # State machine for door frame detection and actions
        if state == "wall_following":
            # Green: Follows the wall
            #
            print(f"Last 10 Door states: {door_states[-80:]}")
            if any(door_states[-80:]):  #looking into the last 10 entries to equal 30cm maybe? to see if any is true(there was a door)
                probability_DD = 0.3     #(probability of door is 0.3 a door was recently seen)
            else:
                probability_DD = 0.7
            if sensor_distances[6] > distance_increase_threshold: 
                probability_D_dist = 0.9 #probability of door is 0.9 if significant distance increase is detected
            else:
                probability_D_dist = 0.2 #else with no distance increase, door probability is 0.2
            
            #probability of door given previous doors and evidence
            probability_D_evidence = probability_DD * probability_D_dist

            #Here we can say if probability_D_evidence is above a threshhold, then proceed to update the state.
            #or we can generate a random number and if it is less than the probability then proceed.

            #2nd option:
            rand_num = random.random()
            print(f"Probability of evidence of a door is {probability_D_evidence}")
            if sensor_distances[6] > distance_increase_threshold: 
            #if rand_num <= probability_D_evidence:
                print(f"probabilities align to say we are in door frame")
                state = "possible_door_frame"
                previous_heading = heading
                distance_travelled_in_frame = 0  # Reset distance tracker 
                print("Transition to Possible Door Frame")
                return "yellow"

                # Default color for wall following
            return "green"

        elif state == "possible_door_frame":
            # Yellow: Possible door frame detected
            distance_travelled_in_frame += math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
            
            if distance_travelled_in_frame > distance_travel_threshold and sensor_distances[6] < distance_decrease_threshold:
                state = "in_door_frame"
                door_start_heading = previous_heading
                print("Transition to In Door Frame")
                return "magenta"

            # Remain yellow during this state
            return "yellow"

        elif state == "in_door_frame":
            # Magenta: In door frame
            current_heading = heading
            if abs(current_heading - door_start_heading) < angle_tolerance:
                state = "past_door_frame"
                doors_passed += 1  # Increment door count
                distance_travelled_after_door = 0  # Reset distance for post-door travel
                print(f"Transition to Past Door Frame. Doors passed: {doors_passed}")
                return "blue"

            # Stay magenta if still in door frame
            return "magenta"

        elif state == "past_door_frame":
            # Blue: After passing door frame, resumes wall-following for 30 cm
            distance_travelled_after_door += math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
            if distance_travelled_after_door >= distance_after_door_threshold:
                if doors_passed >=door_count_goal:
                    state = "stop_and_return_home"  # Trigger return if goal is reached
                    print("Door count goal reached. Preparing to return home.")
                else:
                    state = "wall_following"  # Resume wall-following if not yet reached
                    print("Resuming wall-following after passing door frame")
                return "green"

            # Continue in blue if moving away from door
            return "blue"

        elif state == "stop_and_return_home":
            # Green: Wall-following complete; pause for 5 seconds and return home
            print("Reached stopping point. Pausing for 5 seconds before returning home.")
            await stop_robot()  # Stop in place for 5 seconds
            await asyncio.sleep(5)  # Pause for 5 seconds
            print("Returning to start position.")
            await return_to_start()  # Call return_to_start to send robot home
            state = "returning_home"  # Update state to indicate returning home
            return "red"  # Red to indicate the return-to-home state

        elif state == "returning_home":
            # Final state; ensure the robot stops upon reaching home
            await stop_robot()
            print("Robot returned home and stopped.")
            state = "completed"  # Move to a completed state to prevent further actions
            return "red"

        elif state == "completed":
            # Robot has completed all tasks; stay in idle state
            await stop_robot()
            return "off"  # No further color change; robot remains idle

    # No color change needed if state hasn't updated
    return None
