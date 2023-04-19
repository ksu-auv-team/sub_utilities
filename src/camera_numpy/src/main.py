#!/usr/bin/python3

from numpysocket import NumpySocket
import imutils
import cv2

cap = cv2.VideoCapture(0)

with NumpySocket() as s:
    s.connect(("192.168.1.122", 9999))
    while(cap.isOpened()):
        ret, frame = cap.read()
        frame_resize = imutils.resize(frame, width=640)
        if ret is True:
            try:
                s.sendall(frame_resize)
            except:
                break
        else:
            break