import cv2
vidcap = cv2.VideoCapture('GH010403_Trim.mp4')
print("going")
success,image = vidcap.read()
count = 0
while success and count < 700:
  cv2.imwrite("./pictures/second_frame%d.jpg" % count, image)     # save frame as JPEG file      
  success,image = vidcap.read()
  print('Read a new frame: ', success)
  count += 1
print("done")