
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
parser.add_argument('-t', '--training_set', help="specify set of targets to use")
parser.add_argument('-s', '--state-machine', default="execute_withState", help="set name of state machine to use (default: %(default)s)")
parser.add_argument('-n', '--network', default="run-nnet", help="set name of neural network to use (default: %(default)s)")
parser.add_argument('-v', '--verbosity', help="set logging verbosity (doesn't work)")
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
    for i in range(len(curr_children)):
        proc = curr_children.pop()
        if not proc.poll():
            proc.kill()

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
            print(last_read)
            start()         
            #os.system("roslaunch movement_package manualmode.launch") #restart code from startup
        elif (ArduinoCommand=='0' and last_read == '1'):
            print('read 0')
            last_read = '0'
            print(last_read)
            kill_children()
            print('stopped')

#start ALL the things
def start():
    global rc
    global ssd
    global mv
    global ex
    global curr_children
    global delay_start
    
    #keep logs from each start in a separate directory
    if not args.no_log:
        curr_log_dir = '/home/owl/logs/{}'.format(datetime.datetime.now())
        os.mkdir(curr_log_dir)
        curr_log_dir += '/'


    print('starting roscore')
    with open('{}roscoreout.txt'.format(curr_log_dir), 'w') as rcout:
        rc = subprocess.Popen(['roscore'], stdout=rcout, stderr=rcout)
    curr_children.append(rc)

    delay_start = time.time()
    if not delay_read(10):
        print('starting run-nnet')
        with open('{}ssdout.txt'.format(curr_log_dir), 'w') as ssdout:
            ssd = subprocess.Popen(['python', '/home/owl/src/ncs-ros/{}.py'.format(args.network)], stdout=ssdout, stderr=ssdout)
        curr_children.append(ssd)

        print('starting movement_package')
        with open('{}movementout.txt'.format(curr_log_dir), 'w') as mvout:
            mv = subprocess.Popen(['roslaunch', 'movement_package', 'manualmode.launch'], stdout=mvout, stderr=mvout)    
        curr_children.append(mv)
        delay_start = time.time()
    if not args.manual:    
        if not delay_read(30): #delay to give the pixhawk time to start
            print('starting execute')
            with open('{}execout.txt'.format(curr_log_dir), 'w') as exout:
                ex = subprocess.Popen(['python', '/home/owl/src/subdriver2018/execute_withState.py', args.state_machine], stdout=exout, stderr=exout)
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
                print(last_read)
                kill_children()
                stopped = True
                return stopped
            elif ArduinoCommand == '1':
                last_read = '1'
    return stopped

# hardcoded port number means arduino has to be the second USB device plugged in
ser = serial.Serial('/dev/ttyACM1', 9600, timeout=.001)

#the loop everything runs from
while True:
    listen()



#code for claw control from 2018 competition - probably useless now

# start()
# print('started')
# time.sleep(5)
# stop()
# print('stopped')

# except serial.serialutil.SerialException:
#      #need to find the proper function here (would fix numerous problems)
#     ser.close()
#     ser.open()
#     pass
#print(serial.__version__)

#program for sending servo command 
#run " servo("closeclaw") " will send a serial command to arduino to close claw
#str servo = ''
    # def servocode(servo):
    #     if(servo=='closeclaw'):
    #         ser.write(b'A')
    #         servo=('')
    #     elif(servo=="openclaw"):
    #         ser.write(b'B')
    #         servo=('')
    #     elif(servo=="storeball"):
    #         ser.write(b'C')
    #         servo=('')
    #     elif(servo=="retrieveball"):
    #         ser.write(b'D')
    #         servo=('')  
    #     else:
    #         return;    


    # elif (ser.in_waiting==0):
# servocode(servo='closeclaw') #command for testing communication
