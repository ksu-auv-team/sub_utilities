#!/usr/bin/env python3

import serial #pySerial
import os
import subprocess
import time
import datetime
import argparse
import signal
import sys

class SubSession():
    def __init__(self):
        # Subprocesses:
        self.curr_children = []
        self.rc = None
        self.network = None
        self.mv = None
        self.ex = None
        
        # Arduino variables
        self.delay_start = 0
        self.last_read = '0'        
        
    # shut down child processes for restarting them cleanly or exiting
    def kill_children(self):
        self.curr_children
        print("Removing all processes...")
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

    # listens to the killswitch over serial for state changes and calls start() and kill_children() when necessary
    def listen(self):
        if (ser.in_waiting>=1):
            ArduinoCommand=ser.read()
            # ArduinoCommand = ArduinoCommand.decode('utf-8')
            if (ArduinoCommand=='1' and self.last_read =='0'):
                print('read 1')
                self.last_read = '1'
                self.start()         
                #os.system("roslaunch movement_package manualmode.launch") #restart code from startup
            elif (ArduinoCommand=='0' and self.last_read == '1'):
                print('read 0')
                self.last_read = '0'
                self.kill_children()
                print('stopped')

    #start ALL the things
    def start(self):
        #keep logs from each start in a separate directory
        script_directory = os.path.dirname(os.path.realpath(__file__)) + '/'
        curr_log_dir = script_directory + '../logs/{}/'.format(datetime.datetime.now())
        os.mkdir(curr_log_dir)

        # Create commands to run from argparse
        roscore_command = ['roscore']

        video_string = "python " + script_directory + "pict.py " + args.no_save_images
        video_command = video_string.split()

        network_string = "python3 " + script_directory + '../submodules/jetson_nano_inference/jetson_live_object_detection.py --no-video --model ' + args.network_model + ' ' + args.no_save_images
        network_command = network_string.split()

        movement_string = "roslaunch movement_package manualmode.launch"
        movement_command = movement_string.split()

        execute_string = 'python ' + script_directory + '../submodules/subdriver2018/execute_withState.py --machine ' + args.state_machine + ' ' + args.debug_execute
        execute_command = execute_string.split()

        if(args.no_arduino): # Run the programs without starting up the arduino
            print('starting roscore')
            with open('{}roscoreout.txt'.format(curr_log_dir), 'w') as rcout:
                self.rc = subprocess.Popen(roscore_command, stdout=rcout, stderr=rcout)
            self.curr_children.append(self.rc)
            
            time.sleep(3)
            if (args.no_network):
                print("starting video node")
                with open('{}videoout.txt'.format(curr_log_dir), 'w') as videoout:
                    self.video = subprocess.Popen(video_command, stdout=videoout, stderr=videoout)
                self.curr_children.append(self.video)

            else:
                print('starting Neural Network')
                with open('{}networkout.txt'.format(curr_log_dir), 'w') as networkout:
                    self.network = subprocess.Popen(network_command, stdout=networkout, stderr=networkout)
                self.curr_children.append(self.network)

            time.sleep(3)
            print('starting movement_package')
            with open('{}movementout.txt'.format(curr_log_dir), 'w') as mvout:
                self.mv = subprocess.Popen(movement_command, stdout=mvout, stderr=mvout)    
            self.curr_children.append(self.mv)

            time.sleep(3)
            if not args.manual:
                print('starting execute')
                with open('{}executeout.txt'.format(curr_log_dir), 'w') as executeout:
                    self.ex = subprocess.Popen(execute_command, stdout=executeout, stderr=executeout)
                self.curr_children.append(self.ex)
                
            print('exiting start')

        else: # We do have an arduino hooked up...
            print('starting roscore')
            with open('{}roscoreout.txt'.format(curr_log_dir), 'w') as rcout:
                self.rc = subprocess.Popen(roscore_command, stdout=rcout, stderr=rcout)
            self.curr_children.append(self.rc)

            self.delay_start = time.time()
            if not self.delay_read(10):
                if(not args.no_network):
                    print('starting Neural Network')
                    with open('{}networkout.txt'.format(curr_log_dir), 'w') as networkout:
                        self.network = subprocess.Popen(network_command, stdout=networkout, stderr=networkout)
                    self.curr_children.append(self.network)

                print('starting movement_package')
                with open('{}movementout.txt'.format(curr_log_dir), 'w') as mvout:
                    self.mv = subprocess.Popen(movement_command, stdout=mvout, stderr=mvout)    
                self.curr_children.append(self.mv)
                self.delay_start = time.time()
            if not args.manual:    
                if not self.delay_read(30): #delay to give the pixhawk time to start
                    print('starting execute')
                    with open('{}executeout.txt'.format(curr_log_dir), 'w') as executeout:
                        self.ex = subprocess.Popen(execute_command, stdout=executeout, stderr=executeout)
                    self.curr_children.append(self.ex)
                    
            print('exiting start')

    # listen mid-startup for <duration> seconds to be ready to shut down any existing subprocesses if the switch is turned off
    def delay_read(self, duration):
        stopped = False
        while(time.time() - self.delay_start) < duration:
            if (ser.in_waiting>=1):
                ArduinoCommand=ser.read()
            # ArduinoCommand = ArduinoCommand.decode('utf-8')
                #os.system("roslaunch movement_package manualmode.launch") #restart code from startup
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
        sys.exit(0)

if __name__ == '__main__':
    # Parse command line arguments:
    parser = argparse.ArgumentParser(description="run the submarine")
    parser.add_argument('-i', '--internet-address', help="override default hostname or ip address for remote computer (not currently functional)")
    parser.add_argument('-m', '--manual', action='store_true', help="start in manual mode") #TODO: keep the killswitch from killing programs in manual mode
    parser.add_argument('-d', '--dry-run', action='store_true', help="start as if on land, with video input from a file (not currently functional - may be better implemented with alt. neural network files")
    parser.add_argument('-s', '--state-machine', default="full_state_machine", help="set name of state machine to use (default: %(default)s)")
    parser.add_argument('-n', '--network-model', default="ssd_mobilenet_v1_coco", help="set name of neural network to use (default: %(default)s)")
    parser.add_argument('-v', '--verbosity', help="set logging verbosity (doesn't work)")
    parser.add_argument('--no-arduino', action='store_true', help='Runs the sub without running any physical arduino hardware.')
    parser.add_argument('--no-network', action='store_true', help='Runs the sub without running the neural network')
    parser.add_argument('--no-save-images', action='store_const', default ='', const='--no-save-images', help='Will not record any video/pictures from the sub')
    parser.add_argument('--debug-execute', action='store_const', default='', const='--debug', help='Will run execute with the debug flag')
    args = parser.parse_args()

    # Create Subsession
    go_sub_go = SubSession()

    # captureing Ctrl+C
    signal.signal(signal.SIGINT, go_sub_go.signal_handler)
    
    # hardcoded port number means arduino has to remaped in udev rules to arduino_0
    if not args.no_arduino:
        ser = serial.Serial('/dev/arduino_0', 9600, timeout=.001)

    # If we are running without an arduino hooked up, just run the start, don't listen()
    if args.no_arduino:
        go_sub_go.start()

    #the loop everything runs from
    while True:
        if not args.no_arduino:
            go_sub_go.listen()

