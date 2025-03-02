# 🚧Under construction🚧
Install Visual Studio 
Install Visual Studio Build tools: https://visualstudio.microsoft.com/downloads/

On VS Code, run the command: 
->>> pip install irobot
->>> pip3 install irobot_edu_sdk
->>> python -m venv irobot_env (Create Virtual enironment)
--.....After env is created run python scripts under the directory "Scripts"

## Files

### MTest(1,2*,3).py: 
Main function to 
initiate connection to robot
define robot movement functions ie forward, backoff...
monitor and document percieved robot states ie "Wall following", "in door frame", "exciting door frame"...
write robot sensor readings to file for CPT table computation.

*MTest2.py is the most recent stable version and should be run opposesd to the others

### CPT_Test2:
Wall following functions with pid control. 

### Robot_sensor_data.csv:
Contains sensor, speed, PID, distance, bumper data collected from MTest2.py. A subset of this data will be used in final_cpt_data.csv to calculate conditional probabilities of a door being passed given the bump detected variable (True/False) and a significant change in distance from the wall (True/False)

### Final_cpt_data.csv:
Contains data entries for bump detected, current distance to wall and if a door was present in that timestamp. This data is directly used by final_probability.py to compute the CPT table. This is an aggregation of multiple runs of the robot with preset Door passed values.
The first runs involve just plain walls with no doors to see how the robot's distances change and if there would be any bumping when traversing a straight wall with no doors or obstacles. The second is set of runs is with the robot starting at just the beginning of the door and setting door values to true. This is to test how the robot's distances to the wall changes and bumps when there is guaranteed to be a door in its path. These are the data entries with the door passed set to true. The third set of runs involve the robot traversing a wall with some obstacles but NO doors. This way there are different combinations of when the distance increases and when bumps are detected. Because there is no door present, the door passed values in this run are set to false. The robot however registers multiple bumpsp and distance changes which will be interpreted in the final_probabilty.py file.

### Final_probability.py:
This reads in data from final_cpt_data.csv and takes the data in chunks. The chunk size used was three to simulate 10cm passed. It reads in the data and creates a table in memory. Each three entries in the csv create one entry in the memory table. The memory table consists of three fields namely, door_passed which corresponds to the door passed field in the csv table, the bump which is true if there is a bump in any of the three entries collected in the csv and false otherwise, and distance_changed which is true if the change in distance between the first and third entry is greater than approx 7cm and false otherwise. This was to account for slight fluctuatations the robot would make in simply following a flat, clear wall. Once this table in memory is constructed, Pandas is used to calculate the number of times we have different True/false combinations for bump and distance changed. Given a comination, for example, bump is true and distance changed is false, the probability of door is equal to the number of times that door is true divided by the total number of rows in the table that match this combination. This is done for all four combinations, and values are recorded as the probabilities.

### index(2).html:

Contain html code for project website depicting project descriptions, belief network, cpt table as well as other relevant links and descriptions.



