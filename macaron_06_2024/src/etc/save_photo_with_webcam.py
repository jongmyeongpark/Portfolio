#!/usr/bin/env python
# -*-coding:utf-8-*-
import cv2
import time

cap= cv2.VideoCapture(0)


if cap.isOpened():
    
    while True:
        now = time.time()

        ret,frame = cap.read()
        
        if ret:
            cv2.imshow('camera', frame)

            if cv2.waitKey(1) != -1:
                cv2.imwrite(str(now)+'photo.jpg', frame)
            
        else:
            print('no frame')
            break
else:
    print('no camera!')
cap.release()
cv2.destroyAllWindows()