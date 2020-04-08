#!/usr/bin/env python3
# Sub Vizualizer GUI using PySimpleGUI

import PySimpleGUI as sg   
import sys
import rospy

#Adjust this line to match the location of your gbl.py
# sys.path.append('../subdriver/StateMachine')
sys.path.append('../submodules/subdriver/StateMachine')
# sys.path.append('../sub-utilities/submodules/subdriver/StateMachine')

reload = True

def main():
    import gbl
    global reload
    
    #Import config.txt if it exists
    try:
        file = open("config.txt","r")
        config = eval(file.read())
        file.close()
    except FileNotFoundError:
        rospy.loginfo("config.txt not found. One will be created on GUI exit.")
        config = {}
        
    #Get all variables in gbl, remove the ones we don't care about
    gbl_vars = dir(gbl)
    gbl_vars = [gbl_vars for gbl_vars in gbl_vars if not gbl_vars.startswith('__')]


    try:
        #Collect active topics from ROS
        topics = rospy.get_published_topics()
        #Separate topics and msg types into their own lists.
        #TODO: get real data from ROS, create a big ol switch to handle many msg formats
        msg_Types = list(list(zip(*topics))[1])
        topics = list(list(zip(*topics))[0])
        #Remove backslash in rostopic names. It breaks stuff
        for i in range(len(topics)):
            topics[i] = topics[i].replace('/', '')
            
    except (ConnectionRefusedError , AttributeError) as error:
        rospy.loginfo(error)
        rospy.loginfo('Could not find ROS. Continuing without it.')
        topics = []
    
    
    gbl_commands = []
    #TODO: Dynamically subscribe to n rostopics and get messages.
    for i in topics:
        gbl_commands.append('global {}; {} = "TODO" '.format(i,i))
    
    topics.extend(gbl_vars)
    
    for i in gbl_vars:
        gbl_commands.append('global {}; {} = gbl.{}'.format(i,i,i))
    
    #Import and initialize a blacklist if file is present
    try:
        blacklist_file = open("blacklist.txt","r")
        blacklist = blacklist_file.read().splitlines()
        blacklist_file.close()
    except FileNotFoundError:
        rospy.loginfo("blacklist.txt not found")
        blacklist = []
        
    for i in topics:
        for j in blacklist:
            if i == j:
                topics.pop(topics.index(i))
    
    
    #Create GUI objects based on the topics that were collected
    topic_boxes = []
    topic_text = []
    for i in topics:
        try:
            if config[i]:
                topic_boxes.append([sg.Checkbox('{}'.format(i), enable_events=True, default=True)])
            else:
                topic_boxes.append([sg.Checkbox('{}'.format(i), enable_events=True, default=False)])
        except KeyError:
            topic_boxes.append([sg.Checkbox('{}'.format(i), enable_events=True, default=False)])
            
        topic_text.append(sg.Text('', visible=False, size=(30,1), key='{}'.format(i),auto_size_text=True))
    
    def build_layout(topic_boxes, topic_text):
        #Build the window layout for PySimpleGUI
        layout = [
            [sg.Column(layout=[
                *topic_boxes
                ], scrollable=True, vertical_scroll_only=True,key='col'),
                *topic_text],
            [sg.Exit(), sg.Button('Reload', key='Reload')]
        ]
        return layout
    
    
    #Displays the Window
    window = sg.Window('Sub_Viz', build_layout(topic_boxes, topic_text), resizable=True, finalize=True)
    
    #GUI Event Loop
    #Loop occurrs every time a window.read() happens (either a button is pressed or timeout)
    timeoutcount = 0
    
    while True:
        #timeout=250 -> ms until window updates. Decrease to make GUI 
        #readout more responsive at the cost of CPU time, and vise versa
        tout = 250
        event, values = window.read(timeout=tout, timeout_key='Timeout') 
        # rospy.loginfo(window['col'].get_size())
        # rospy.loginfo(window.size)
        #Importing gbl again updates its values? Need to verify...
        import gbl
        for i in gbl_commands:
            exec(i)
        if event == 'Timeout':
            for i in range(len(values)):
                if values[i] == True:
                    try:
                        if (timeoutcount*(tout/1000) > 10):  
                            window[topics[i]].update('{}: {}'.format(topics[i], eval(topics[i])),visible=True)
                        else:
                            window[topics[i]].update('{}: {}'.format(topics[i], eval(topics[i])),visible=True)
                    except: 
                        window[topics[i]].update('{}: ERROR'.format(topics[i]))
                else:
                    window[topics[i]].update(visible=False)
        window['col'].expand(False,True)
        
        if event == ('Reload'):
            reload = True 
            break
        if event in (None, 'Exit'):
            reload = False
            break
        
    # event, values = window.read()
    window.close()
    for i in range(len(topics)):
        values['{}'.format(topics[i])] = values.pop(i)
    file = open("config.txt","w+")
    file.write(str(values))
    file.close()
    

if __name__ == '__main__':
    while reload:
        main()


