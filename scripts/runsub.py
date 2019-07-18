#!/usr/bin/env python3

import serial #pySerial
import os
import subprocess
import time
import datetime
import argparse
import signal
import sys
import rospy
from std_msgs.msg import Bool

class SubSession():
    def __init__(self, no_arduino=False):
        # Subprocesses:
        self.curr_children = []
        self.startup_processes = []
        
        # Arduino variables
        self.delay_start = 0
        self.sub_is_killed = True
        self.no_arduino = no_arduino

        # Ros init
        self.killswitch_sub = rospy.Subscriber("killswitch_is_killed", Bool, self.killswitch_callback)

        #keep logs from each start in a separate directory
        self.script_directory = os.path.dirname(os.path.realpath(__file__)) + '/'
        self.curr_log_dir = self.script_directory + '../logs/{}/'.format(datetime.datetime.now())
        os.mkdir(self.curr_log_dir)
        
    # shut down child processes for restarting them cleanly or exiting
    def kill_children(self):
        self.curr_children
        print("Removing all runtime processes...")
        for i in range(len(self.curr_children)):
            try:
                proc = self.curr_children.pop()
                print("Killing: ")
                print(proc)
                if not proc.poll():
                    proc.kill()
            except Exception as e:
                print(e)
        print("Done!")

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
            print("Removed Startup Process!")
        bashCommand = "pkill -f ros"
        process = subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE) 

    # listens to the killswitch over serial for state changes and calls start() and kill_children() when necessary
    def listen(self):
        if (ser.in_waiting>=1):
            ArduinoCommand=ser.read().decode('utf-8',errors='ignore')
            if (ArduinoCommand=='1' and self.last_read =='0'):
                print('read 1')
                self.last_read = '1'
                self.start()         
            elif (ArduinoCommand=='0' and self.last_read == '1'):
                print('read 0')
                self.last_read = '0'
                self.kill_children()
                print('stopped')

    def start_roscore(self):
        roscore_command = ['roscore']

        print('starting roscore')
        with open('{}roscoreout.txt'.format(self.curr_log_dir), 'w') as rcout:
            rc = subprocess.Popen(roscore_command, stdout=rcout, stderr=rcout)
            return rc

    def start_video(self):
        video_string = "python " + self.script_directory + "pict.py " + args.no_save_images
        video_command = video_string.split()

        print("starting video node")
        with open('{}videoout.txt'.format(self.curr_log_dir), 'w') as videoout:
            video = subprocess.Popen(video_command, stdout=videoout, stderr=videoout)
            return video

    def start_network(self):
        network_string = "python3 " + self.script_directory + '../submodules/jetson_nano_inference/jetson_live_object_detection.py --no-video --model ' + args.network_model + ' ' + args.no_save_images
        network_command = network_string.split()
    
        print('starting Neural Network')
        with open('{}networkout.txt'.format(self.curr_log_dir), 'w') as networkout:
            network = subprocess.Popen(network_command, stdout=networkout, stderr=networkout)
            return network

    def start_movement(self):
        movement_string = "roslaunch movement_package manualmode.launch"
        movement_command = movement_string.split()

        print('starting movement_package')
        with open('{}movementout.txt'.format(self.curr_log_dir), 'w') as mvout:
            mv = subprocess.Popen(movement_command, stdout=mvout, stderr=mvout)
            return mv 

    def start_execute(self):
        execute_string = 'python ' + self.script_directory + '../submodules/subdriver2018/execute_withState.py --machine ' + args.state_machine + ' ' + args.debug_execute
        execute_command = execute_string.split()

        print('starting execute')
        with open('{}executeout.txt'.format(self.curr_log_dir), 'w') as executeout:
            ex = subprocess.Popen(execute_command, stdout=executeout, stderr=executeout)
            return ex
                
    def start_arduino(self):
        arduino_string = "rosrun rosserial_python serial_node.py /dev/arduino_0"
        arduino_command = arduino_string.split()

        print('starting Arduino Process')
        with open('{}arduinoout.txt'.format(self.curr_log_dir), 'w') as ardout:
            arduino = subprocess.Popen(arduino_command, stdout=ardout, stderr=ardout)
            return arduino

    #start ALL the things
    def start(self):     
        rate = rospy.Rate(10)

        # Run the Video Node
        self.curr_children.append(self.start_video())
        
        self.delay_start = time.time() # The time we will compare our arduino time to
        while(time.time() - self.delay_start < 10 and not self.sub_is_killed):
            rospy.spin()

        # Run Movement Package
        self.curr_children.append(self.start_movement())

        self.delay_start = time.time() # The time we will compare our arduino time to
        while(time.time() - self.delay_start < 10) and not self.sub_is_killed:
            rospy.spin()

        # Run Execute
        if(args.manual):
            print('Manual Mode enabled, start your joystick node')
        else:
            self.curr_children.append(self.start_execute())

        '''
        self.delay_start = time.time() # The time we will compare our arduino time to

        # Run Video / Network Commands:
        run_video = True
        if not args.no_arduino:
            run_video = not self.delay_read(10)
        else:
            time.sleep(10)
        if (args.no_network and run_video):
            self.curr_children.append(self.start_video())
        elif (run_video):
            self.curr_children.append(self.start_network())
        else:
            return

        self.delay_start = time.time() # The time we will compare our arduino time to
        
        # Run Movement Package
        run_movement = True
        if not args.no_arduino:
            run_movement = not self.delay_read(10)
        else:
            time.sleep(10)
        if (run_movement):   
            self.curr_children.append(self.start_movement())
        else:
            return
        

        self.delay_start = time.time() # The time we will compare our arduino time to

        # Run Execute
        run_execute = True
        if not args.no_arduino:
            run_execute = not self.delay_read(20)
        else:
            time.sleep(20)
        if (not args.manual and run_execute):
            self.curr_children.append(self.start_execute())
        elif (args.manual):
            print('Manual Mode')
        else:
            return
        '''
                
        print('exiting start')

    # listen mid-startup for <duration> seconds to be ready to shut down any existing subprocesses if the switch is turned off
    def delay_read(self, duration):
        stopped = False
        while(time.time() - self.delay_start) < duration:
            if (ser.in_waiting>=1):
                ArduinoCommand=ser.read().decode('utf-8',errors='ignore')
                if (ArduinoCommand=='0' and self.last_read == '1'):
                    print('read 0')
                    self.last_read = '0'
                    self.kill_children()
                    stopped = True
                    return stopped
                elif ArduinoCommand == '1':
                    self.last_read = '1'
        return stopped

    def signal_handler(self, sig, frame):
        print("\nCaptured Ctrl+C, stopping execution...")
        self.kill_children()
        if not self.no_arduino:
            self.kill_startup()
        sys.exit(0)

    def killswitch_callback(self, msg):
        if (msg.data and self.sub_is_killed==False):
            print('Sub has been killed')
            self.sub_is_killed = True
            self.kill_children()
        elif (not msg.data and self.sub_is_killed==True):
            print('Starting Sub Runtime Processes')
            self.sub_is_killed = False
            self.start()

if __name__ == '__main__':
    # Parse command line arguments:
    parser = argparse.ArgumentParser(description="run the submarine")
    parser.add_argument('-i', '--internet-address', help="override default hostname or ip address for remote computer (not currently functional)")
    parser.add_argument('-m', '--manual', action='store_true', help="start in manual mode") #TODO: keep the killswitch from killing programs in manual mode
    parser.add_argument('-d', '--dry-run', action='store_true', help="start as if on land, with video input from a file (not currently functional - may be better implemented with alt. neural network files")
    parser.add_argument('-s', '--state-machine', default="BaseStateMachine", help="set name of state machine to use (default: %(default)s)")
    parser.add_argument('-n', '--network-model', default="ssd_mobilenet_v1_coco", help="set name of neural network to use (default: %(default)s)")
    parser.add_argument('-v', '--verbosity', help="set logging verbosity (doesn't work)")
    parser.add_argument('--no-arduino', action='store_true', help='Runs the sub without running any physical arduino hardware.')
    parser.add_argument('--no-network', action='store_true', help='Runs the sub without running the neural network')
    parser.add_argument('--no-save-images', action='store_const', default ='', const='--no-save-images', help='Will not record any video/pictures from the sub')
    parser.add_argument('--debug-execute', action='store_const', default='', const='--debug', help='Will run execute with the debug flag')
    args = parser.parse_args()

    # Create Subsession
    go_sub_go = SubSession(args.no_arduino)

    # captureing Ctrl+C
    signal.signal(signal.SIGINT, go_sub_go.signal_handler)

    # Wait for arduino to start
    time.sleep(3)
    
    # If we are running without an arduino hooked up, just run the start, don't listen()
    if args.no_arduino:
        go_sub_go.startup_processes.append(go_sub_go.start_roscore())
        go_sub_go.start()

    # If we do have an arduino hooked up, we need to forward the ROS stuff over
    else:
        go_sub_go.startup_processes.append(go_sub_go.start_roscore())
        go_sub_go.startup_processes.append(go_sub_go.start_arduino())
        if(not args.no_network):
            go_sub_go.startup_processes.append(go_sub_go.start_network())

    rospy.init_node("run_sub")

    #the loop everything runs from
    rospy.spin()
