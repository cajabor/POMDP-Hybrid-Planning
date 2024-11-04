# trace_ui.py

import tkinter as tk
import socket
import threading

# Network configuration
HOST = '127.0.0.1'
PORT = 65432

# Initialize tkinter window
root = tk.Tk()
root.title("Room Tracing")
canvas_size = 600  # Canvas size
canvas_center = canvas_size // 2
initial_scale = 5  # Initial scale factor

# Set up canvas
canvas = tk.Canvas(root, width=canvas_size, height=canvas_size, bg="white")
canvas.pack()

# Colors for the paths
ROBOT_COLOR = "blue"
WALL_COLOR = "red"

# Robot path and position data
robot_position = [canvas_center, canvas_center]  # Start in the center
previous_position = robot_position.copy()
path_data = []  # Store all received (x, y) positions to redraw when zooming
wall_data = []  # Store wall (x, y) positions
scale = initial_scale

# Offsets for panning
offset_x = 0
offset_y = 0

# Function to update UI with new robot and wall positions
def update_ui(robot_x, robot_y, Senor6_wall_x, Senor6_wall_y):
    global previous_position, path_data, wall_data
    # Store received positions in path data for redrawing
    path_data.append((robot_x, robot_y))
    wall_data.append((Senor6_wall_x, Senor6_wall_y))
    draw_path()

# Function to draw the paths based on the current scale and offsets
def draw_path():
    global previous_position
    # Clear canvas and reset previous_position to center
    canvas.delete("all")
    previous_position = [canvas_center + offset_x, canvas_center + offset_y]

    # Draw robot path in blue
    for x, y in path_data:
        canvas_x = canvas_center + (x * scale) + offset_x
        canvas_y = canvas_center - (y * scale) + offset_y  # Invert y for display
        canvas.create_line(previous_position[0], previous_position[1], canvas_x, canvas_y, fill=ROBOT_COLOR)
        previous_position = [canvas_x, canvas_y]

    # Draw wall path in red
    previous_position = [canvas_center + offset_x, canvas_center + offset_y]
    for wx, wy in wall_data:
        wall_canvas_x = canvas_center + (wx * scale) + offset_x
        wall_canvas_y = canvas_center - (wy * scale) + offset_y  # Invert y for display
        canvas.create_line(previous_position[0], previous_position[1], wall_canvas_x, wall_canvas_y, fill=WALL_COLOR)
        previous_position = [wall_canvas_x, wall_canvas_y]

# Zoom in function (increase scale)
def zoom_in(event=None):
    global scale
    scale += 1
    draw_path()

# Zoom out function (decrease scale)
def zoom_out(event=None):
    global scale
    if scale > 1:  # Prevent scale from going below 1
        scale -= 1
        draw_path()

# Panning functions
def pan_left(event=None):
    global offset_x
    offset_x += 10
    draw_path()

def pan_right(event=None):
    global offset_x
    offset_x -= 10
    draw_path()

def pan_up(event=None):
    global offset_y
    offset_y += 10
    draw_path()

def pan_down(event=None):
    global offset_y
    offset_y -= 10
    draw_path()

# Bind mouse scroll for zooming
canvas.bind("<MouseWheel>", lambda event: zoom_in() if event.delta > 0 else zoom_out())

# Create control frame for layout organization
control_frame = tk.Frame(root)
control_frame.pack()

# Zoom control frame
zoom_frame = tk.Frame(control_frame)
zoom_frame.grid(row=0, column=0, padx=10)

# Direction control frame
direction_frame = tk.Frame(control_frame)
direction_frame.grid(row=0, column=1, padx=10)

# Zoom buttons in the zoom_frame
zoom_in_button = tk.Button(zoom_frame, text="+", command=zoom_in)
zoom_in_button.grid(row=0, column=0)

zoom_out_button = tk.Button(zoom_frame, text="-", command=zoom_out)
zoom_out_button.grid(row=1, column=0)

# Directional buttons in the direction_frame
up_button = tk.Button(direction_frame, text="↑", command=pan_up)
up_button.grid(row=0, column=1)

left_button = tk.Button(direction_frame, text="←", command=pan_left)
left_button.grid(row=1, column=0)

right_button = tk.Button(direction_frame, text="→", command=pan_right)
right_button.grid(row=1, column=2)

down_button = tk.Button(direction_frame, text="↓", command=pan_down)
down_button.grid(row=2, column=1)

# Socket listener to receive position updates
def listen_for_updates():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind((HOST, PORT))
        while True:
            data, _ = s.recvfrom(1024)
            robot_x, robot_y, wall_x, wall_y = map(float, data.decode().split(","))
            update_ui(robot_x, robot_y, wall_x, wall_y)

# Start the socket listener in a separate thread
listener_thread = threading.Thread(target=listen_for_updates, daemon=True)
listener_thread.start()

# Run tkinter main loop
root.mainloop()
