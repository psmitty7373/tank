#!/usr/bin/python3

import cv2, time, imutils, io
from picamera.array import PiRGBArray
from picamera import PiCamera
from imutils import contours
import numpy as np

t_count = 0
class tank:
    def __init__(self, x=0, y=0):
        global t_count
        self.pos = [x, y]
        self.last_update = time.time()
        self.bit_sequence = [0]
        self.id = t_count
        t_count += 1

    def update(self, x, y):
        self.pos = [x, y]
        self.last_update = time.time()
        self.bit_sequence[-1] = 1

tanks = []

def find_possible_tank(x, y):
    found = False
    global tanks
    for t in tanks:
        is_close = np.isclose(t.pos, [x,y], 0.2)
        if is_close[0] and is_close[1]:
            return t
    return found

hsv = None
lower = None
upper = None

def pick_color(event, x, y, flags, param):
    global hsv
    global lower
    global upper
    if event == cv2.EVENT_LBUTTONDOWN:
        pixel = hsv[y,x]
        #you might want to adjust the ranges(+-10, etc):
        upper =  np.array([pixel[0] + 10, pixel[1] + 10, pixel[2] + 40])
        lower =  np.array([pixel[0] - 10, pixel[1] - 10, pixel[2] - 40])
        print(pixel, lower, upper)

#        image_mask = cv2.inRange(image_hsv,lower,upper)
#        cv2.imshow("mask",image_mask)

stream_done = False

def outputs():
    fw = (800 + 31)//32*32
    fh = (600 + 15)//16*16
    stream = io.BytesIO()
    last = time.time()
    tot_time = 0
    frames = 0
    while not stream_done:
        yield stream
        stream.seek(0)
        Y=np.fromstring(stream.getvalue(), dtype=np.uint8, count=fw*fh).reshape((fh,fw))
        stream.seek(0)
        stream.truncate()
        now = time.time()
        tot_time += now - last
        frames += 1
        last = now
#        print(1 / (tot_time / frames))

def find_tanks():
    global hsv
    global lower
    global upper
    cam = cv2.VideoCapture(0)
    cam.set(cv2.CAP_PROP_FRAME_WIDTH, 1024)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 768)
    cam.set(cv2.CAP_PROP_FPS, 40)

#    camera = PiCamera()
#    camera.resolution = (800, 600)
#    camera.framerate = 40
#    camera.exposure_mode = 'off'
#    camera.awb_mode = 'off'
#    camera.awb_gains = 1
#    camera.shutter_speed = 1500
#    raw_cap = PiRGBArray(camera)
    time.sleep(1)

    last_frame = time.time()
    last_print = last_frame
    frames = 0
    tot_time = 0
    cv2.namedWindow('hsv')
    cv2.setMouseCallback('hsv', pick_color)
    while True:
        for t in tanks:
            t.bit_sequence.append(0)
            t.bit_sequence = t.bit_sequence[-12:]

        ret, image = cam.read()
        #camera.capture_sequence(outputs(), format="bgr", use_video_port=True)
        #image = raw_cap.array
        now = time.time()
#        blurred = cv2.GaussianBlur(image, (5, 5), 0)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv,(0, 0, 255), (30, 30, 255))
        mask = cv2.inRange(hsv,(0, 0, 255), (30, 30, 255))
#        cv2.erode(mask, np.ones((3, 3), dtype=np.uint8))
#        cv2.dilate(mask, np.ones((5, 5), dtype=np.uint8))

        cnts = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        found = False
        if len(cnts) > 0:
            for (i, c) in enumerate(cnts):
                found = True
                ((cX, cY), radius) = cv2.minEnclosingCircle(c)
                if radius > 1.0:
                    t = find_possible_tank(cX, cY)
                    if not t:
                        tanks.append(tank(cX, cY))
                    else:
                        t.update(cX, cY)
#                    if t.pos[0] > 100:
#                        print(t.id, t.pos, t.bit_sequence)


#                print(cX, cY)
#                cv2.circle(image, (int(cX), int(cY)), int(radius), (0, 0, 255), 3)
#                cv2.putText(image, "#{}".format(i + 1), (int(cX), int(cY) - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 2)


        tot_time += now - last_frame
        frames += 1
        last_frame = now

        cv2.imshow("hsv", image)
        if now - last_print > 5:
            print(1 / (tot_time / frames))
            last_print = now

        if cv2.waitKey(1) == 27: 
            break  # esc to quit
        #raw_cap.truncate(0) 
    cv2.destroyAllWindows()

def main():
    find_tanks()

if __name__ == '__main__':
    main()
