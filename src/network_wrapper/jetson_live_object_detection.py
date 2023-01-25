#!/usr/bin/env python3

import tensorflow as tf
import numpy as np
import argparse
import cv2
import tensorflow.contrib.tensorrt as trt
import time
import sys
import signal
from .src.object_detector import ObjectDetection
import os
import datetime
from submarine_msgs_srvs.msg import Detections


""" Jetson Live Object Detector """
class JetsonLiveObjectDetection():
    def __init__(self, model, camera=None, debug=False, thresh=0.4, last_network_callback_time=0.0):
        self.debug = debug
        if test_video_picture is not None:
            self.camera = cv2.VideoCapture(test_video_picture)    
        elif self.debug:    
            self.camera = cv2.VideoCapture(camera)
            self.camera.set(3, args.width)
            self.camera.set(4, args.height)
        self.model = model
        self.detector = ObjectDetection(self.model, label_map=args.label)
        self.thresh = float(thresh)
        self.last_network_callback_time = last_network_callback_time

        # Default to not running the network node on either camera, must push an enable/disable message from statemachine
        # We do this so we can control what states the network is actually running in 
        self.run_network_front = False
        self.run_network_bottom = False

    def signal_handler(self, sig, frame):
        ''' Handles interrupt signals from OS.

        Args:
            signal: signal received from system
            frame: state capture for diagnostic porpises.
        '''
        cv2.destroyAllWindows()
        if args.debug or test_video_picture is not None:
            self.camera.release()
        self.detector.__del__()
        sys.exit(0)

    def _visualizeDetections(self, img, scores, boxes, classes, num_detections):
        ''' Draws detections on images and returns a list of the detections name in string form 

        Args:
            img: OpenCV Mat, The current OpenCV Image
            scores: Float List, List of confidence values 
            boxes: 2D Float List, Bounding boxes for the objects
            classes: Float List, List of the IDs for each detected object
            num_detections: Float/Int, Number of detections found
        
        Returns:
            img: OpenCV Mat, Newly drawn-on image
            detections: String List, Actual names of object detected in a list
        '''

        cols = img.shape[1]
        rows = img.shape[0]
        detections = []

        for i in range(num_detections):
            bbox = [float(p) for p in boxes[i]]
            score = float(scores[i])
            classId = int(classes[i])
            if score > self.thresh:
                detections.append(self.detector.labels[str(classId)])
                x = int(bbox[1] * cols)
                y = int(bbox[0] * rows)
                right = int(bbox[3] * cols)
                bottom = int(bbox[2] * rows)
                thickness = int(4 * score)
                cv2.rectangle(img, (x, y), (right, bottom), (125,255, 21), thickness=thickness)
                cv2.putText(img, self.detector.labels[str(classId)] + ': ' + str(round(score,3)), (x,y), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)

        return img, detections

    def static_video(self):
        ''' Run the object detection on recorded video or images
        '''
        if test_video_picture is not None:
            fourcc = cv2.VideoWriter_fourcc('M','J','P','G')
            names = test_video_picture.split('.')
            if args.test_video is not None and not args.no_save_images:
                out = cv2.VideoWriter(names[0] + '_output.' + names[1], fourcc, 30.0, (640,480))
            while(self.camera.isOpened()):
                ret, img = self.camera.read()
                if ret:
                    scores, boxes, classes, num_detections = self.detector.detect(img)
                    img, new_detections = self._visualizeDetections(img, scores, boxes, classes, num_detections)
                    if (args.show_video):
                        cv2.imshow(names[0], img)
                        if cv2.waitKey(1) & 0xFF == ord('q'):
                            break
                    if args.test_video is not None and not args.no_save_images:
                        out.write(img)
                    elif args.test_picture is not None:
                        cv2.imwrite(names[0] + '_output.' + names[1], img)
                    print(("Found objects: " + str(' '.join(new_detections)) + "."))
                else:
                    break
            if args.test_video is not None and not args.no_save_images:
                out.release()
            self.camera.release()
            self.detector.__del__()
            if (not args.no_save_images):
                print(("Output File written to " + names[0] + "_output." + names[1]))
            exit()

    def start(self):
        ''' Starts up the detector for static or live video.
        '''
        img_counter = 0
        frame_counter = 0

        if (not args.no_save_images):
            script_directory = os.path.dirname(os.path.realpath(__file__)) + '/'
            save_dir = script_directory + 'saved_video/{}_{}/'.format(self.model,datetime.datetime.now())
            os.mkdir(save_dir)

        print ("Starting Live object detection, may take a few minutes to initialize...")
        self.detector.initializeSession()
        
        # Run the code with static video/picture testing:
        if test_video_picture is not None:
           self.static_video() 

        # Run the code in DEBUG mode. That is, with a local camera.
        elif self.debug:  
            # Health Checks:
            if not self.camera.isOpened():
                print ("Camera has failed to open")
                exit(-1)

            fps = self.camera.get(cv2.CAP_PROP_FPS)

            # Main Programming Loop
            while True:
                curr_time = time.time()

                ret, img = self.camera.read()
                if (frame_counter >= fps / args.rate):
                    frame_counter = 0
                    scores, boxes, classes, num_detections = self.detector.detect(img)
                    new_detections = None
                    img, new_detections = self._visualizeDetections(img, scores, boxes, classes, num_detections)
            
                    print(("Found objects: " + str(' '.join(new_detections)) + "."))
                    if (args.show_video):
                        cv2.imshow('Jetson Live Detection', img)
                    if ((img_counter % 3) == 0 and not args.no_save_images):
                        img_name = "{}opencv_frame_{}.jpg".format(save_dir, int(curr_time))
                        cv2.imwrite(img_name, img)
                        img_counter = 0
                
                    img_counter += 1

                    # Publish ros-bridged images
                    if not args.no_ros:
                        img_msg = bridge.cv2_to_imgmsg(img)
                        front_img_pub.publish(img_msg)

                        detections_msg = Detections()
                        detections_msg.scores = scores
                        detections_msg.boxes = boxes.flatten()
                        detections_msg.classes = classes
                        detections_msg.detected = [num_detections]
                        front_detections_pub.publish(detections_msg)

                    print(("Network running at: " + str(1.0/(time.time() - curr_time)) + " Hz."))

                frame_counter += 1
            
                if cv2.waitKey(1) == ord('q'):
                    break

            cv2.destroyAllWindows()
            self.camera.release()
            self.detector.__del__()
            print ("Exiting...")
            return

        # Run the code as a ROS node, pulls images on a topic, published them out on antoher
        else:
            rospy.Subscriber('front_raw_imgs', Image, self.run_network_node_front, queue_size=1)
            #rospy.Subscriber('bottom_raw_imgs', Image, self.run_network_node_bottom, queue_size=1)
            rospy.Subscriber('enable_front_network', Bool, self.enable_front_callback)
            rospy.Subscriber('enable_bottom_network', Bool, self.enable_bottom_callback)
            rospy.spin()

    def run_network_node_front(self, msg):
        ''' Runs network node on the recevial of an image from ROS 

        Args:
            msg: ROS OpenCV Brige message, image to process on 
        '''
        
        # Disable the front camera check
        if not self.run_network_front:
            return

        if (time.time() - self.last_network_callback_time) <= args.rate:
            return
        bridge = cv_bridge.CvBridge()
        img = bridge.imgmsg_to_cv2(msg)

        scores, boxes, classes, num_detections = self.detector.detect(img)
        new_detections = None
        img, new_detections = self._visualizeDetections(img, scores, boxes, classes, num_detections)
        print(("Found Front objects: " + str(' '.join(new_detections)) + "."))

        img_msg = bridge.cv2_to_imgmsg(img)
        front_img_pub.publish(img_msg)

        detections_msg = Detections()
        detections_msg.scores = scores
        detections_msg.boxes = boxes.flatten()
        detections_msg.classes = classes
        detections_msg.detected = [num_detections]
        front_detections_pub.publish(detections_msg)

        self.last_network_callback_time = time.time()

    def run_network_node_bottom(self, msg):
        ''' Runs network node on the recevial of an image from ROS 

        Args:
            msg: ROS OpenCV Brige message, image to process on 
        '''

        # Disable the bottom camera check
        if not self.run_network_bottom:
            return

        if (time.time() - self.last_network_callback_time) <= args.rate:
            return
        bridge = cv_bridge.CvBridge()
        img = bridge.imgmsg_to_cv2(msg)

        scores, boxes, classes, num_detections = self.detector.detect(img)
        new_detections = None
        img, new_detections = self._visualizeDetections(img, scores, boxes, classes, num_detections)
        print(("Found Bottom objects: " + str(' '.join(new_detections)) + "."))

        img_msg = bridge.cv2_to_imgmsg(img)
        #bottom_img_pub.publish(img_msg)

        detections_msg = Detections()
        detections_msg.scores = scores
        detections_msg.boxes = boxes.flatten()
        detections_msg.classes = classes
        detections_msg.detected = [num_detections]
        bottom_detections_pub.publish(detections_msg)

        self.last_network_callback_time = time.time()

    def enable_front_callback(self, msg):
        self.run_network_front = msg.data

    def enable_bottom_callback(self, msg):
        self.run_network_bottom = msg.data

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="This script runs inference on a trained object detection network")
    parser.add_argument('-m', '--model', default="ssd_mobilenet_v1_coco", help="set name of neural network model to use")
    parser.add_argument('-v', '--verbosity', action='store_true', help="set logging verbosity (doesn't work)")
    parser.add_argument('-d', '--debug', action='store_true', help='Runs the network using a local camera, not from ROS, but will still publish to ROS topics.')
    parser.add_argument('-c', '--camera', default='/dev/video0', help='/path/to/video, defaults to /dev/video0')
    parser.add_argument('--height', default=420, help='Set the video capture height for your camera in pixels')
    parser.add_argument('--width', default=860, help='Set the video capture width for your camera in pixels')
    parser.add_argument('-r', '--rate', type=int, default=-1, help='Specify the rate to run the neural network at, i.e. number of images to look at per second. Defaults to fastests possible.')
    parser.add_argument('-l', '--label', default='label_map.pbtxt', help='Override the name of the label map in your model directory. Defaults to label_map.pbtxt')
    parser.add_argument('--test-video', help='/path/to/test/video This is used if you want to test your network on a static video. It will append \'_output\' to your file before saving it.')
    parser.add_argument('--test-picture', help='/path/to/test/picture This is used if you want to test your network on a static picture. It will append \'_output\' to your file before saving it.')
    parser.add_argument('--thresh', default=0.4, help='Override the default detection threshold. Default = 0.4')
    parser.add_argument('--show-video', action='store_true', help='Will display live video feed on local machine.')
    parser.add_argument('--no-save-images', action='store_true', help='Will not record any video/pictures from the sub')
    parser.add_argument('--no-ros', action='store_true', help='Will not subscribe or publish to any ros topics')
    parser.add_argument('--front-start-on', action='store_true', help='Will start with the front camera node running')
    parser.add_argument('--bottom-start-on', action='store_true', help='Will start with the bottom camera node running')

    if args.no_ros:
        args = parser.parse_args()

    if not args.no_ros:
        import cv_bridge
        import rospy
        from sensor_msgs.msg import Image
        from std_msgs.msg import Bool
        bridge = cv_bridge.CvBridge()

        args = parser.parse_args(rospy.myargv()[1:])

        rospy.init_node('Network_Vision')
        
        front_img_pub = rospy.Publisher('front_network_imgs', Image, queue_size=1)
        front_detections_pub = rospy.Publisher('front_network_output', Detections, queue_size=1)
        
        #bottom_img_pub = rospy.Publisher('bottom_network_imgs', Image, queue_size=1)
        #bottom_detections_pub = rospy.Publisher('bottom_network_output', Detections, queue_size=1)

    test_video_picture = None
    if args.test_video is not None and args.test_picture is not None:
        print("Please don't use --test_video and --test_picture at the same time.")
        exit()
    elif args.test_video is not None:
        test_video_picture = args.test_video
    elif args.test_picture is not None:
        test_video_picture = args.test_picture

    live_detection = JetsonLiveObjectDetection(model=args.model, camera=args.camera, debug=args.debug, thresh=args.thresh, last_network_callback_time=time.time())

    live_detection.run_network_front = args.front_start_on
    live_detection.run_network_bottom = args.front_start_on

    # captureing Ctrl+C
    signal.signal(signal.SIGINT, live_detection.signal_handler)

    live_detection.start()
    

