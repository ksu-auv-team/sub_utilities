#!/usr/bin/env python2

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from Tkinter import Tk, Canvas, BOTH, PhotoImage, StringVar, Label

class App:
    def __init__(self, master):
        self.master = master
        master.title("ROS Camera Viewer")

        self.canvas = Canvas(master, width=1280, height=720)
        self.canvas.pack(fill=BOTH, expand=1)

        # self.label_text = StringVar()
        # self.label = Label(master, textvariable=self.label_text)
        # self.label.pack()

        # self.label_text_2 = StringVar()
        # self.label_2 = Label(master, textvariable=self.label_text_2)
        # self.label_2.pack()

        # self.label_text_3 = StringVar()
        # self.label_3 = Label(master, textvariable=self.label_text_3)
        # self.label_3.pack()

        self.bridge = CvBridge()
        print("1")
        self.image_sub = rospy.Subscriber("/usb_cam_cam/image_raw", Image, self.image_callback)
        print("2")
        self.current_frame = None

        # self.data = rospy.Subscriber("/data", StringVar, self.grab_data)

        master.after(10, self.update_image)
        # self.update_label()

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            print("Received image with size: {}".format(cv_image.shape))
            self.current_frame = cv_image
        except CvBridgeError as e:
            print(e)

    def update_image(self):
        if self.current_frame is not None:
            cv2_image = cv2.cvtColor(self.current_frame, cv2.COLOR_BGR2RGBA)
            img = cv2.resize(cv2_image, (640, 480))
            photo = PhotoImage(image=cv2ImageTk.PhotoImage(img))
            self.canvas.create_image(0, 0, image=photo, anchor="nw")
            self.canvas.image = photo
        self.master.after(10, self.update_image)

    def grab_data(self):
        # Grab data from ROS
        data = rospy.get_param("/data")
        return data

    def update_label(self):
        # get updated data
        data = self.grab_data()
        data = [0, 1, 2]
        # parse data
        temp = "Temperature: {}".format(data[0])
        hum = "Humidity: {}".format(data[1])
        depth = "Depth: {}".format(data[2])

        # update labels
        self.label_text.set(temp)
        self.label_text_2.set(hum)
        self.label_text_3.set(depth)

        # update every second
        self.root.after(1000, self.update_label)

def main():
    rospy.init_node("ros_camera_viewer", anonymous=True)
    root = Tk()
    app = App(root)
    root.mainloop()

if __name__ == "__main__":
    main()
