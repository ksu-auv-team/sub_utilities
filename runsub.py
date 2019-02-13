
"""
Created on Tue Jul 31 18:41:01 2018
 
@author: Tripp
"""
#int ArduinoCommand
import serial
import os
import subprocess
import time
import atexit

rc = None
ssd = None
mv = None
ex = None

curr_children = []

delay_start = 0

last_read = '0'



def kill_children():
    global curr_children
    for i in range(len(curr_children)):
        proc = curr_children.pop()
        if not proc.poll():
            proc.kill()

atexit.register(kill_children)

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

def start():
    global rc
    global ssd
    global mv
    global ex
    global curr_children
    global delay_start
    print('starting roscore')
    with open('logs/roscoreout{}.txt'.format(str(time.time())[-7:]), 'w') as rcout:
        rc = subprocess.Popen(['roscore'], stdout=rcout, stderr=rcout)
    curr_children.append(rc)
    delay_start = time.time()
    if not delay_read(10):
        print('starting run-nnet')
        with open('logs/ssdout{}.txt'.format(str(time.time())[-7:]), 'w') as ssdout:
            ssd = subprocess.Popen(['python', '/home/owl/src/ncs-ros/run-nnet.py'], stdout=ssdout, stderr=ssdout)
        curr_children.append(ssd)
        print('starting movement_package')
        with open('logs/movementout{}.txt'.format(str(time.time())[-7:]), 'w') as mvout:
            mv = subprocess.Popen(['roslaunch', 'movement_package', 'manualmode.launch'], stdout=mvout, stderr=mvout)    
        curr_children.append(mv)
        delay_start = time.time()
        if not delay_read(30):
            print('starting execute')
            with open('logs/execout{}.txt'.format(str(time.time())[-7:]), 'w') as exout:
                ex = subprocess.Popen(['python', '/home/owl/src/subdriver2018/execute.py'], stdout=exout, stderr=exout)
            curr_children.append(ex)
            print('exiting start')


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

# try:
ser = serial.Serial('/dev/ttyACM1', 9600, timeout=.001)

while True:
    listen()

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