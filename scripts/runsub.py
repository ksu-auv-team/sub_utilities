#!/usr/bin/env python

# launching things
import os
import subprocess
import sys
import shutil

# Waiting for things
import time
import datetime

# Parse command line
import argparse

# Capture ctrl+c
import signal

# ROS imports
import rospy
import roslaunch
from std_msgs.msg import Bool

class SubSession():
    def __init__(self, state_machine, arbitrary_machine, network_model, no_save_images, no_network, debug_execute, manual=False, simulated=False):
        #Arguments
        self.manual_ = manual
        self.state_machine_ = state_machine
        self.arbitrary_machine_ = arbitrary_machine
        self.network_model_ = network_model
        self.no_save_images_ = no_save_images
        self.no_network_ = no_network
        self.debug_execute_ = debug_execute
        self.simulated = simulated

        # Subprocesses:
        self.launcher = roslaunch.scriptapi.ROSLaunch()
        self.launcher.start()
        self.movement_node = None
        self.simulation_process = None
        self.curr_children = []
        self.startup_processes = []
        
        # Arduino variables
        self.delay_start = 0
        self.sub_is_killed = False

        #keep logs from each start in a separate directory
        self.script_directory = os.path.dirname(os.path.realpath(__file__)) + '/'
        self.curr_log_dir = self.script_directory + '../logs/{}/'.format(datetime.datetime.now())
        os.mkdir(self.curr_log_dir)
        
        # ROS subscribers
        killswitch_start_sub = rospy.Subscriber("killswitch_run_start", Bool, self.killswitch_start_callback)
        killswitch_realtime_sub = rospy.Subscriber("killswitch_is_killed", Bool, self.killswitch_realtime_callback, queue_size=1)

    # shut down child processes for restarting them cleanly or exiting
    def kill_children(self):
        self.curr_children
        print("Removing all runtime processes...")
        for child in self.curr_children:
            try:
                child.stop()
            except Exception as e:
                print(e)
        del self.curr_children[:]
        
        # Because manualmode is a launch file, we have to stop it separately
        try:
            self.movement_node.shutdown()
        except Exception as e:
            print(e)

        print("Done!")

    # Kills the processes that are tied to startup_processes
    def kill_startup(self):
        print("Removing Startup Process...")
        for i in range(len(self.startup_processes)):
            try:
                proc = self.startup_processes.pop()
                print("Killing: ")
                print(proc)
                if not proc.poll():
                    proc.kill()
            except Exception as e:
                print(e)

        # Attempt to kill the simulator
        if self.simulation_process is not None:
            try:
                self.simulation_process.kill()
            except Exception as e:
                print(e)
        
        print("Removed Startup Process!")

    def start_video(self):
        rospy.loginfo("Starting the video node")

        # Get a node reference
        package = 'camera_utilities'
        executable = 'camera_node.py'
        video = roslaunch.core.Node(package, executable, args=self.no_save_images_)

        # Launch the node
        video_node = self.launcher.launch(video)

        # Return the reference to the node
        return video_node

    def start_network(self):
        rospy.loginfo("Starting the network node")

        # Get a node reference
        package = 'network_wrapper'
        executable = 'jetson_live_object_detection.py'
        network = roslaunch.core.Node(package, executable, args="--model " + self.network_model_ + " " + self.no_save_images_)

        # Launch the node
        network_node = self.launcher.launch(network)

        # Return the reference to the node
        return network_node

    def start_movement(self):
        rospy.loginfo("Starting the movement package")

        # Get a reference to the launch file
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        if(not self.simulated):
            movement_launch = roslaunch.parent.ROSLaunchParent(uuid, [self.script_directory + "../src/movement_package/launch/manualmode.launch"])
        else:
            # 
            simulation_string = "sim_vehicle.py -v ArduSub -L Transdec --map --console"
            simulation_commands = simulation_string.split()
            self.simulation_process = subprocess.Popen(simulation_commands, stdout=subprocess.PIPE)
            time.sleep(20) # Wait for the simulator to start up
            movement_launch = roslaunch.parent.ROSLaunchParent(uuid, [self.script_directory + "../src/movement_package/launch/simulated_mode.launch"])
        
        movement_launch.start()

        # return that reference
        return movement_launch

    def start_execute(self):
        rospy.loginfo("Starting the subdriver node")

        # Get a node reference
        package = 'subdriver'
        executable = 'execute_withState.py'
        arg_string = "--machine " + self.state_machine_ + " " + self.debug_execute_

        # Add on the arbitrary machine that will override the regular state machine
        if (self.arbitrary_machine_):
            arg_string += " -a " + self.arbitrary_machine_

        execute = roslaunch.core.Node(package, executable, args=arg_string)

        # Launch the node
        execute_node = self.launcher.launch(execute)

        # Return the reference to the node
        return execute_node
                
    def start_arduino(self):
        rospy.loginfo("Starting the arduino node")

        # Get a node reference
        package = 'rosserial_python'
        executable = 'serial_node.py'
        arduino = roslaunch.core.Node(package, executable, args="/dev/arduino_0")

        # Launch the node
        arduino_node = self.launcher.launch(arduino)

        # Return the reference to the node
        return arduino_node

    #start ALL the things
    def start(self):     
        # Run the Video Node
        if not self.no_save_images_ and not self.no_network_:
            self.curr_children.append(self.start_video())
        
        self.delay_start = time.time() # The time we will compare our arduino time to
        while(time.time() - self.delay_start < 2 and not self.sub_is_killed):
            pass

        # Run Movement Package
        self.movement_node = self.start_movement()

        self.delay_start = time.time() # The time we will compare our arduino time to
        while(time.time() - self.delay_start < 25 and not self.sub_is_killed):
            pass

        # Run Execute
        if(self.manual_):
            rospy.loginfo('Manual Mode enabled, start your joystick node')
        else:
            self.curr_children.append(self.start_execute())
        
        rospy.loginfo('exiting start')

    # This function captures the Ctrl+C and exits the function cleanly
    def signal_handler(self, sig, frame):
        print("\nCaptured Ctrl+C, stopping execution...")
        self.kill_children()
        self.kill_startup()

        # Kill roscore
        bashCommand = "pkill -f ros"
        process = subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE) 
        
        # Copy the logs to a local folder
        self.copytree(os.path.expanduser("~") + "/.ros/log/latest", self.curr_log_dir)

        # Exit the script
        sys.exit(0)

    # This callback triggers on a CHANGE in the killswitch, so 0->1 or 1->0. It starts the process.
    def killswitch_start_callback(self, msg):
        if(msg.data):
            rospy.loginfo('Starting Sub Runtime Processes')
            self.start()
        else:
            rospy.loginfo('Sub has been killed')
            self.kill_children()
            
    # This killswitch triggers every time step from the arduino in real time.
    def killswitch_realtime_callback(self, msg):
            self.sub_is_killed = msg.data

    # Used for copying logs
    def copytree(self, src, dst, symlinks=False, ignore=None):
        for item in os.listdir(src):
            s = os.path.join(src, item)
            d = os.path.join(dst, item)
            if os.path.isdir(s):
                shutil.copytree(s, d, symlinks, ignore)
            else:
                shutil.copy2(s, d)
        

def create_args():
	# Parse command line arguments:
	parser = argparse.ArgumentParser(description="run the submarine")
	parser.add_argument('-m', '--manual', action='store_true', help="Will not run state machine")
	parser.add_argument('-s', '--state-machine', default="QualifyStraightMachine", help="set name of state machine to use (default: %(default)s)")
	parser.add_argument('-a', '--arbitrary-machine', default="", help="Run an arbitrary state (This will override --state-machine). Pass in the Python Import structure as if it were being run from subdriver. EX: StateMachine.taskless.dumb_start.Dumb_Start")
	parser.add_argument('-n', '--network-model', default="qual_2_rcnn_frozen", help="set name of neural network to use (default: %(default)s)")
	parser.add_argument('-d', '--debug-execute', action='store_const', default='', const='--debug', help='Will run execute with the debug flag')
	parser.add_argument('--simulate', action='store_true', help='Will also start the simulator node instead of being hooked up to hardware')
	parser.add_argument('--no-arduino', action='store_true', help='Runs the sub without running any physical arduino hardware.')
	parser.add_argument('--no-network', action='store_true', help='Runs the sub without running the neural network')
	parser.add_argument('--no-save-images', action='store_const', default ='', const='--no-save-images', help='Will not record any video/pictures from the sub')
	parser.add_argument('--start-front-network', action='store_true', help='Will begin with the front neural network running')
	parser.add_argument('--start-bottom-network', action='store_true', help='Will begin with the bottom neural network running')
	return parser.parse_args(rospy.myargv()[1:])

if __name__ == '__main__':
    # Create args
    args = create_args()

    # Start roscore
    roscore = subprocess.Popen('roscore')
    time.sleep(7)  # wait a bit to be sure the roscore is really launched

    # Create Subsession
    go_sub_go = SubSession(args.state_machine, args.arbitrary_machine, args.network_model, args.no_save_images, args.no_network, args.debug_execute, args.manual, args.simulate)

    # captureing Ctrl+C
    signal.signal(signal.SIGINT, go_sub_go.signal_handler)
    
    # Ros init
    rospy.init_node("run_sub")

    # By default, the State Machine is enabling/disabling the neural network on the front and bottom cameras
    # Here, we can override that by enabling the front or bottom at the beginning
    if not args.no_network:
        if args.start_front_network:
            front_pub = rospy.Publisher('enable_front_network', Bool, queue_size=1)
            front_pub.publish(True)
        if args.start_bottom_network:
            bottom_pub = rospy.Publisher('enable_bottom_network', Bool, queue_size=1)
            bottom_pub.publish(True)
    
    # If we are running without an arduino hooked up, just run the start, don't wait for the killswitch to be pressed
    if args.no_arduino:
        if(not args.no_network):
            go_sub_go.startup_processes.append(go_sub_go.start_network())
        time.sleep(3)
        go_sub_go.start()

    # If we do have an arduino hooked up, we need to forward the ROS stuff over and listen for the killswitch
    else:
        go_sub_go.startup_processes.append(go_sub_go.start_arduino())
        if(not args.no_network):
            go_sub_go.startup_processes.append(go_sub_go.start_network())

    # The loop everything runs from
    rospy.spin()