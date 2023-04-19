#!/bin/sh

# The below code is to be executed on the surface station

# Run the joystick node
screen -S tf2 rosrun joy joy_node

# Run the camera server python script
screen -S tf2 python3 src/camera_numpy/src/main.py