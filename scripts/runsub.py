
"""
Created on Tue Jul 31 18:41:01 2018
"""

import serial #pySerial
import os
import subprocess
import atexit
import time
import datetime
import argparse

parser = argparse.ArgumentParser(description="run the submarine")
parser.add_argument('-i', '--internet-address', help="override default hostname or ip address for remote computer (not currently functional)")
parser.add_argument('-m', '--manual', action='store_true', help="start in manual mode") #TODO: keep the killswitch from killing programs in manual mode
parser.add_argument('-d', '--dry-run', action='store_true', help="start as if on land, with video input from a file (not currently functional - may be better implemented with alt. neural network files")
parser.add_argument('-s', '--state-machine', default="full_state_machine", help="set name of state machine to use (default: %(default)s)")
parser.add_argument('-n', '--network-model', default="ssd_mobilenet_v1_coco", help="set name of neural network to use (default: %(default)s)")
parser.add_argument('-v', '--verbosity', help="set logging verbosity (doesn't work)")
parser.add_argument('--no-arduino', action='store_true', help='Runs the sub without running any physical arduino hardware.')
parser.add_argument('--no-network', action='store_true', help='Runs the sub without running the neural network')
args = parser.parse_args()

# future subprocesses
rc = None
ssd = None
mv = None
ex = None

#list of running subprocesses
curr_children = []

#time to begin listening during delay
delay_start = 0

#last character read over serial connection
last_read = '0'

# shut down child processes for restarting them cleanly or exiting
def kill_children():
    global curr_children
    print("Removing all processes...")
    for i in range(len(curr_children)):
        proc = curr_children.pop()
        if not proc.poll():
            proc.kill()
    print("Done!")

#shut down all subprocesses when program is exited
atexit.register(kill_children)

# listens to the killswitch over serial for state changes and calls start() and kill_children() when necessary
def listen():
    global last_read
    if (ser.in_waiting>=1):
        ArduinoCommand=ser.read()
        # ArduinoCommand = ArduinoCommand.decode('utf-8')
        if (ArduinoCommand=='1' and last_read =='0'):
            print('read 1')
            last_read = '1'
            start()         
            #os.system("roslaunch movement_package manualmode.launch") #restart code from startup
        elif (ArduinoCommand=='0' and last_read == '1'):
            print('read 0')
            last_read = '0'
            kill_children()
            print('stopped')

#start ALL the things
def start():
    global rc
    global network
    global mv
    global ex
    global curr_children
    global delay_start
    
    #keep logs from each start in a separate directory
    script_directory = os.path.dirname(os.path.realpath(__file__)) + '/'
    curr_log_dir = script_directory + '../logs/{}/'.format(datetime.datetime.now())
    os.mkdir(curr_log_dir)

    if(args.no_arduino):
        print('starting roscore')
        with open('{}roscoreout.txt'.format(curr_log_dir), 'w') as rcout:
            rc = subprocess.Popen(['roscore'], stdout=rcout, stderr=rcout)
        curr_children.append(rc)

        if (not args.no_network):
            print('starting Neural Network')
            with open('{}networkout.txt'.format(curr_log_dir), 'w') as networkout:
                network = subprocess.Popen(['python3', script_directory + '../submodules/jetson_nano_inference/jetson_live_object_detection.py', '--model {}'.format(args.network_model)], stdout=networkout, stderr=networkout)
            curr_children.append(network)

        print('starting movement_package')
        with open('{}movementout.txt'.format(curr_log_dir), 'w') as mvout:
            mv = subprocess.Popen(['roslaunch', 'movement_package', 'manualmode.launch'], stdout=mvout, stderr=mvout)    
        curr_children.append(mv)

        if not args.manual:
            print('starting execute')
            with open('{}executeout.txt'.format(curr_log_dir), 'w') as executeout:
                ex = subprocess.Popen(['python', script_directory + '../submodules/subdriver2018/execute_withState.py', '--machine ' + args.state_machine], stdout=executeout, stderr=executeout)
            curr_children.append(ex)
            print('exiting start')

    else: # We do have an arduino hooked up...
        print('starting roscore')
        with open('{}roscoreout.txt'.format(curr_log_dir), 'w') as rcout:
            rc = subprocess.Popen(['roscore'], stdout=rcout, stderr=rcout)
        curr_children.append(rc)

        delay_start = time.time()
        if not delay_read(10):
            if(not args.no_network):
                print('starting Neural Network')
                with open('{}networkout.txt'.format(curr_log_dir), 'w') as networkout:
                    network = subprocess.Popen(['python3', script_directory + '../submodules/jetson_nano_inference/jetson_live_object_detection.py', '--model {}'.format(args.network_model)], stdout=networkout, stderr=networkout)
                curr_children.append(network)

            print('starting movement_package')
            with open('{}movementout.txt'.format(curr_log_dir), 'w') as mvout:
                mv = subprocess.Popen(['roslaunch', 'movement_package', 'manualmode.launch'], stdout=mvout, stderr=mvout)    
            curr_children.append(mv)
            delay_start = time.time()
        if not args.manual:    
            if not delay_read(30): #delay to give the pixhawk time to start
                print('starting execute')
                with open('{}executeout.txt'.format(curr_log_dir), 'w') as executeout:
                    ex = subprocess.Popen(['python', script_directory + '../submodules/subdriver2018/execute_withState.py', '--machine ' + args.state_machine], stdout=executeout, stderr=executeout)
                curr_children.append(ex)
                print('exiting start')

# listen mid-startup for <duration> seconds to be ready to shut down any existing subprocesses if the switch is turned off
def delay_read(duration):
    global delay_start
    global last_read
    stopped = False
    while(time.time() - delay_start) < duration:
        if (ser.in_waiting>=1):
            ArduinoCommand=ser.read()
        # ArduinoCommand = ArduinoCommand.decode('utf-8')
            #os.system("roslaunch movement_package manualmode.launch") #restart code from startup
            if (ArduinoCommand=='0' and last_read == '1'):
                print('read 0')
                last_read = '0'
                kill_children()
                stopped = True
                return stopped
            elif ArduinoCommand == '1':
                last_read = '1'
    return stopped

if __name__ == '__main__':
    # hardcoded port number means arduino has to remaped in udev rules to arduino_0
    if not args.no_arduino:
        ser = serial.Serial('/dev/arduino_0', 9600, timeout=.001)

    # If we are running without an arduino hooked up, just run the start, don't listen()
    if args.no_arduino:
        start()

    #the loop everything runs from
    while True:
        if not args.no_arduino:
            listen()

