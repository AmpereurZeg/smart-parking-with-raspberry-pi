######################################################################
##################            IMPORTS         ########################
######################################################################
import cv2
import imutils
import numpy as np
import pytesseract
from PIL import Image
from picamera.array import PiRGBArray
from picamera import PiCamera
import RPi.GPIO as GPIO
import time
import sqlite3
import RPi.GPIO as GPIO
import time
######################################################################
##################            IMPORTS         ########################
######################################################################
# GPIO Mode (BOARD / BCM)
GPIO.setmode(GPIO.BCM)

# set GPIO Pins
GPIO_TRIGGER = 18
GPIO_ECHO = 24
GPIO_TRIGGER2 = 17
GPIO_ECHO2 = 23

# set GPIO direction ultrason1 (IN / OUT)
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)
# set GPIO direction ultrason2 (IN / OUT)
GPIO.setup(GPIO_TRIGGER2, GPIO.OUT)
GPIO.setup(GPIO_ECHO2, GPIO.IN)


def ReadPlateNumber():
    camera = PiCamera()
    camera.resolution = (640, 480)
    camera.framerate = 32
    rawCapture = PiRGBArray(camera, size=(640, 480))

    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        image = frame.array
        cv2.imshow("Frame", image)
        key = cv2.waitKey(1) & 0xFF
        rawCapture.truncate(0)
        dist = distance(GPIO_TRIGGER, GPIO_ECHO)
        if dist < 5:
            key = ord('s')
        if key == ord('s'):
            # convert to grey scale
            time.sleep(2)
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            gray = cv2.bilateralFilter(
                gray, 11, 17, 17)  # Blur to reduce noise
            edged = cv2.Canny(gray, 30, 200)  # Perform Edge detection
            cnts = cv2.findContours(
                edged.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            cnts = imutils.grab_contours(cnts)
            cnts = sorted(cnts, key=cv2.contourArea, reverse=True)[:10]
            screenCnt = None
            for c in cnts:
                peri = cv2.arcLength(c, True)
                approx = cv2.approxPolyDP(c, 0.018 * peri, True)
                if len(approx) == 4:
                    screenCnt = approx
                    break
            if screenCnt is None:
                detected = 0
                print("No contour detected")
            else:
                detected = 1
            if detected == 1:
                cv2.drawContours(image, [screenCnt], -1, (0, 255, 0), 3)
            mask = np.zeros(gray.shape, np.uint8)
            new_image = cv2.drawContours(mask, [screenCnt], 0, 255, -1,)
            new_image = cv2.bitwise_and(image, image, mask=mask)
            (x, y) = np.where(mask == 255)
            (topx, topy) = (np.min(x), np.min(y))
            (bottomx, bottomy) = (np.max(x), np.max(y))
            Cropped = gray[topx:bottomx+1, topy:bottomy+1]

            # Read the number plate
            text = pytesseract.image_to_string(Cropped, config='--psm 11')

            return text.strip()
 # x==> trigger
 # y==> echo


def distance(x, y):
    # set Trigger to HIGH
    GPIO.output(x, True)

    # set Trigger after 0.01ms to LOW
    time.sleep(0.00001)
    GPIO.output(x, False)

    StartTime = time.time()
    StopTime = time.time()

    # save StartTime
    while GPIO.input(y) == 0:
        StartTime = time.time()

    # save time of arrival
    while GPIO.input(y) == 1:
        StopTime = time.time()

    # time difference between start and arrival
    TimeElapsed = StopTime - StartTime
    # multiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    distance = (TimeElapsed * 34300) / 2

    return distance


text = ReadPlateNumber()

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(19, GPIO.OUT)
p = GPIO.PWM(19, 50)

print("Detected Number is:", text)

conn = sqlite3.connect('matricule.db')
c = conn.cursor()
c.execute("SELECT * FROM plaque WHERE matricule= '" + text + "'")
d = c.fetchone()
conn.commit()
conn.close()
if d is None:
    detected1 = 0
    print("dont open")
else:
    detected1 = 1
    print("open door ")

if(detected1):
    p.start(2.5)
    p.ChangeDutyCycle(7.5)
    time.sleep(10)
    dist1 = distance(GPIO_TRIGGER, GPIO_ECHO)
    if dist1 > 100 :
        p.ChangeDutyCycle(2.5)
        time.sleep(3)
        p.stop
