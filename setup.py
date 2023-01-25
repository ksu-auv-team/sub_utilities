#!/usr/bin/env python3

import os

# Get the script directory
script_directory = os.path.dirname(os.path.realpath(__file__)) + '/'

# Add easy commands for reading logs
os.system('printf "\n\n# AUV log commands " >> ~/.bashrc')
os.system("printf \"\nalias auv-tail-execute='tail -f ~/.ros/log/latest/execute_withState*-stdout.log'\" >> ~/.bashrc")
os.system("printf \"\nalias auv-tail-camera='tail -f ~/.ros/log/latest/camera_node*-stdout.log'\" >> ~/.bashrc")
os.system("printf \"\nalias auv-tail-network='tail -f ~/.ros/log/latest/jetson_live_object_detection*-stdout.log'\" >> ~/.bashrc")

# Add sourcing the setup.bash to ~/.basrh
os.system('printf "\n\n# Source AUV setup.bash " >> ~/.bashrc')
os.system("printf '\nsource " + script_directory + "devel/setup.bash' >> ~/.bashrc")

# Let the user know it sucessfully executed
print("All done, now please run:\nsource ~/.bashrc")
