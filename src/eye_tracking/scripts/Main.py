#!/usr/bin/env python3
import os
import math
import sys
import time
import numpy as np
import cv2
import dlib
import glob
import rospy
from eye_tracking.msg import eye_position

from FpsManager import FpsManager
from CalibrationManager import CalibrationManager
from PerspectiveTransformManager import PerspectiveTransformManager

from FaceDetector import FaceDetector
from LandmarkDetector import LandmarkDetector

from EyePositionCalculator import EyePositionCalculator


# Constant
MY_PATH = "/home/park/gobbledegook-1.01/src/eye_tracking/scripts/"

CAM_INDEX = 0

ORIGIN_FRAME_WIDTH, ORIGIN_FRAME_HEIGHT = 1280, 720
TARGET_FPS = 60

BOARD_SCALE = 0.8
BOARD_WIDTH, BOARD_HEIGHT = 790, 1090

BOARD_POINTS = (
    (566, 50),
    (1027, 31),
    (564, 654),
    (1021, 676),
)

RESIZE_SCALE = 0.1

FRAME_WIDTH, FRAME_HEIGHT = int(BOARD_WIDTH * BOARD_SCALE),int(BOARD_HEIGHT * BOARD_SCALE)

# Manager
fpsManager = FpsManager()
calibrationManager = CalibrationManager(glob.glob(MY_PATH + 'CalibrationImage/*.jpg'), ORIGIN_FRAME_WIDTH, ORIGIN_FRAME_HEIGHT)
perspectiveTransformManager = PerspectiveTransformManager(BOARD_POINTS, FRAME_WIDTH, FRAME_HEIGHT)

# Face Detector
CASCADE = cv2.CascadeClassifier(MY_PATH + "AddOn/haarcascade_frontalface_default.xml")
faceDetector = FaceDetector(CASCADE, RESIZE_SCALE)

# Landmark Detector
LANDMARK_COUNT = 5
PREDICTOR = dlib.shape_predictor(MY_PATH + "AddOn/shape_predictor_5_face_landmarks.dat")
landmarkDetector = LandmarkDetector(PREDICTOR, LANDMARK_COUNT)

# Eye Position Calculator
EYE_POSITION_INDEXES = (
    # On screen
    2,  # Out side of left eye
    3,  # In side of left eye
    1,  # In side of right eye 
    0,  # Out side of right eye 
)

eyePositionCalculator = EyePositionCalculator(FRAME_WIDTH, FRAME_HEIGHT, BOARD_WIDTH, BOARD_HEIGHT, EYE_POSITION_INDEXES)

# Window
mainWindowName = "main"
cv2.namedWindow(mainWindowName, cv2.WINDOW_NORMAL)
def printMousePos(event, x, y, f, a):
    if event == cv2.EVENT_LBUTTONDOWN:
        print(f"{x}, {y}")

cv2.setMouseCallback(mainWindowName, printMousePos)

# Capture
capture = cv2.VideoCapture(CAM_INDEX)
capture.set(cv2.CAP_PROP_FRAME_WIDTH, ORIGIN_FRAME_WIDTH)
capture.set(cv2.CAP_PROP_FRAME_HEIGHT, ORIGIN_FRAME_HEIGHT)
capture.set(cv2.CAP_PROP_FPS, TARGET_FPS)

# ROS
PUBLISHER_NAME = "eye_position"
PUBLISHER = rospy.Publisher(PUBLISHER_NAME, eye_position, queue_size=10)
rospy.init_node('eye_position', anonymous=True)
PUBLISHER_RATE = rospy.Rate(30)

# Main Loop
def mainLoop():
    global perspectiveTransformManager

    while True:
        # Read frame
        ret, originFrame = capture.read()
        if ret is False:
            print("")
            break

        # Flip frame
        flipFrame = cv2.flip(originFrame, 1)

        # Calibration frame
        calFrame = calibrationManager.calibration(flipFrame)

        # Perspective transform
        perFrame = perspectiveTransformManager.perspectiveTransfrom(calFrame)

        frame = perFrame

        # Gray frame
        grayFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Reduce frame size
        resizedGrayFrame = cv2.resize(grayFrame, dsize=(0,0), fx=RESIZE_SCALE, fy=RESIZE_SCALE, interpolation=cv2.INTER_LINEAR)
        
        # Detect face
        faces = faceDetector.detect(resizedGrayFrame)
        for face in faces:
            (x, y, w, h) = face
            x = int(x / RESIZE_SCALE)
            y = int(y / RESIZE_SCALE)
            w = int(w / RESIZE_SCALE)
            h = int(h / RESIZE_SCALE)
            upscaledFace = (x, y, w, h)

            # Draw face rectangle
            cv2.rectangle(frame, (x, y), (x + w, y + h), (255,0,0), 2)

            # Detect landmarks
            points = landmarkDetector.detect(grayFrame, upscaledFace)

            # Calculate eye position
            eyePositionOnBoard = eyePositionCalculator.calc(points, debugFrame=frame)
            (eyePositionX, eyePositionY) = eyePositionOnBoard

            # print(eyePositionOnBoard)
            if not rospy.is_shutdown():
                msg = eye_position()
                msg.x = eyePositionX
                msg.y = eyePositionY
                PUBLISHER.publish(msg)

        # Update fps
        fpsManager.updateFps()
        fps = fpsManager.getFps()
        cv2.putText(frame, f"FPS: {fps}", (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        # Show frame
        cv2.imshow(mainWindowName, frame)
        cv2.imshow("resizedGrayFrame", resizedGrayFrame)

        # Wait
        inputKey = cv2.waitKey(1)
        
        # Key event
        if inputKey == ord('q'):
            break

        elif inputKey == ord('p'):
            # Find board from calFrame
            perspectiveTransformManager = PerspectiveTransformManager(points, BOARD_WIDTH, BOARD_HEIGHT)

if __name__ == "__main__":
    MY_PATH = "./"

os.system("v4l2-ctl -d 0 --set-ctrl exposure_auto_priority=0")

mainLoop()

cv2.destroyAllWindows()
sys.exit()
