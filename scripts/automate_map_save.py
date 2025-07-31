#!/usr/bin/env python3

import os
import subprocess

WORKSPACE = os.path.expanduser('~/test_ws')
MAP_FOLDER = os.path.join(WORKSPACE, 'src/homebot_0/maps')
MAP_NAME = input("Enter map name (default: my_map): ").strip() or "my_map"
MAP_PATH = os.path.join(MAP_FOLDER, MAP_NAME)

# 1. Make sure the folder exists
os.makedirs(MAP_FOLDER, exist_ok=True)

print("==== 1. Start mapping in a new terminal! (slam_toolbox/rviz/teleop etc) ====")
input("When mapping is COMPLETE and the robot has covered all areas, press Enter to SAVE the map...")

# 2. Save the map
print(f"==== 2. Saving map to {MAP_PATH}.yaml and {MAP_PATH}.pgm ... ====")
subprocess.run([
    "ros2", "run", "nav2_map_server", "map_saver", "-f", MAP_PATH
])

print("==== 3. Map saved! Update nav2_params.yaml automatically... ====")
PARAM_FILE = os.path.join(WORKSPACE, "src/homebot_0/config/nav2_params.yaml")

with open(PARAM_FILE, "r") as f:
    lines = f.readlines()

with open(PARAM_FILE, "w") as f:
    for line in lines:
        if "yaml_filename:" in line:
            # Replace the yaml_filename line
            f.write(f'        yaml_filename: "{MAP_PATH}.yaml"\n')
        else:
            f.write(line)

print("==== 4. All set! You can now launch nav2 navigation: ====")
print("    ros2 launch homebot_0 nav2_launch.py")

