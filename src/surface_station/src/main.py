#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import Tkinter as tk
from PIL import ImageTk, Image as PilImage
import io

class ROSCameraViewer:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)
        self.current_frame = None

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.current_frame = cv_image
        except CvBridgeError as e:
            print(e)

    def get_frame(self):
        return self.current_frame

def main():
    rospy.init_node('ros_camera_viewer', anonymous=True)
    viewer = ROSCameraViewer()

    root = tk.Tk()
    root.title("ROS Camera Viewer")

    frame = tk.Frame(root)
    frame.pack()

    label = tk.Label(frame)
    label.pack()

    def update_image():
        cv_img = viewer.get_frame()
        if cv_img is not None:
            img = PilImage.fromarray(cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB))
            imgtk = ImageTk.PhotoImage(image=img)
            label.imgtk = imgtk
            label.configure(image=imgtk)

        root.after(20, update_image)

    def on_closing():
        root.destroy()

    update_image()
    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.mainloop()

if __name__ == '__main__':
    main()