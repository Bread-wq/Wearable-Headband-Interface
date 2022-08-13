import pyrealsense2 as rs
import time
import numpy as np
import cv2
pipe = rs.pipeline()
profile = pipe.start()
curr_time = time.time()
last_time = time.time()

while 1:
    curr_time = time.time()
    if curr_time - last_time > 1.0:
        print(curr_time-last_time)
        frames = pipe.wait_for_frames()
        #print(frames[1].profile)
        c = np.asanyarray(frames[1].get_data())
        c = cv2.cvtColor(c, cv2.COLOR_RGB2BGR)
        c = cv2.rotate(c, cv2.cv2.ROTATE_90_CLOCKWISE)

        #cv2.imshow("camera", c)
        #cv2.waitKey(1)
        last_time = curr_time