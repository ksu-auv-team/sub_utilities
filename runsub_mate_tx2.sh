 #!/bin/sh

# The below code is to be executed on the TX2

# Load a screen and execute the runsub python file with arguments for pure manual mode
screen -S tf2 ./scripts/runsub.py --no-save-images --no-network --no-arduino --manual

# Load a screen and execute the camera_numpy code to stream cameras
screen -S tf2 python3 src/camera_numpy/src/main.py
