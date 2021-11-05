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
import multiprocessing
import ctypes

from FpsManager import FpsManager
from CalibrationManager import CalibrationManager
from PerspectiveTransformManager import PerspectiveTransformManager
from BoardManager import BoardManager

from FaceDetector import FaceDetector
from LandmarkDetector import LandmarkDetector

from EyePositionCalculator import EyePositionCalculator

from Log import Log

# Constant
MY_PATH = "/home/park/gobbledegook-1.01/src/eye_tracking/scripts/"

# Run with ROS
RUN_WITH_ROS = True

if len(sys.argv) >= 2 and sys.argv[1] == "ROS":
    from eye_tracking.msg import eye_position
else:
    RUN_WITH_ROS = False
    MY_PATH = "./"
    
# Camera
CAM_INDEX = 0
ORIGIN_FRAME_WIDTH, ORIGIN_FRAME_HEIGHT = 1280, 720
TARGET_FPS = 60
capture = cv2.VideoCapture(CAM_INDEX)
capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
capture.set(cv2.CAP_PROP_FRAME_WIDTH, ORIGIN_FRAME_WIDTH)
capture.set(cv2.CAP_PROP_FRAME_HEIGHT, ORIGIN_FRAME_HEIGHT)
capture.set(cv2.CAP_PROP_FPS, TARGET_FPS)

# FPS
fpsManager = FpsManager()

# Calibaration
calibrationManager = CalibrationManager(glob.glob(MY_PATH + "CalibrationImage/*.jpg"), ORIGIN_FRAME_WIDTH, ORIGIN_FRAME_HEIGHT)

# Perspective Transform
boardManager = BoardManager(MY_PATH + "setting.ini")
BOARD_POINTS = boardManager.getBoardEdgePostionArr()
(BOARD_WIDTH, BOARD_HEIGHT) = boardManager.getBoardSize()
BOARD_SCALE = 0.8
FRAME_WIDTH, FRAME_HEIGHT = int(float(BOARD_WIDTH) * BOARD_SCALE), int(float(BOARD_HEIGHT) * BOARD_SCALE) # final frame size

perspectiveTransformManager = PerspectiveTransformManager(BOARD_POINTS, FRAME_WIDTH, FRAME_HEIGHT)

# Face Detector
CASCADE = cv2.CascadeClassifier(MY_PATH + "AddOn/haarcascade_frontalface_default.xml")
RESIZE_SCALE = 0.2  # 0.1
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

eyePositionCalculator = EyePositionCalculator(FRAME_WIDTH, FRAME_HEIGHT, BOARD_WIDTH, BOARD_HEIGHT, EYE_POSITION_INDEXES, MY_PATH + "setting.ini")

# Window
mainWindowName = "main"
cv2.namedWindow(mainWindowName, cv2.WINDOW_NORMAL)

# ROS
def publish(eyePosition):
    PUBLISHER_NAME = "eye_position"
    publisher = rospy.Publisher(PUBLISHER_NAME, eye_position, queue_size=10)
    rospy.init_node('eye_position', anonymous=True)
    publisher_rate = rospy.Rate(30)

    msg = eye_position()
    while not rospy.is_shutdown():
        msg.x = eyePosition[0]
        msg.y = eyePosition[1]
        publisher.publish(msg)
        publisher_rate.sleep()
    
# Main Loop
def mainLoop(eyePosition):
    global perspectiveTransformManager
    SHOW_RESULT = True

    eyePositionOnBoard = np.array((0, 0))

    eyePositionList = []
    measuredTime = 0
    isMeasuring = True
    originEyePosition = eyePositionCalculator.getOriginEyePosition()

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
            eyePosition[0] = eyePositionOnBoard[0] - originEyePosition[0]
            eyePosition[1] = eyePositionOnBoard[1] - originEyePosition[1]
            # (eyeX, eyePositionY) = eyePositionOnBoard
            # eyePosition[0] = eyePositionX
            # eyePosition[1] = eyePositionY
            break

        # Draw board's edge
        boardManager.drawBoardEnge(calFrame)

        # Update fps
        fpsManager.updateFps()
        fps = fpsManager.getFps()
        cv2.putText(frame, f"FPS: {fps}", (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        # Measure origin eye positions
        if isMeasuring:
            measuredTime += fpsManager.getDeltaTime()
            eyePositionList.append(eyePositionOnBoard)

            if measuredTime >= 3000:
                result = np.array((0, 0));
                for pos in eyePositionList:
                    result += pos
                result = result / np.array((len(eyePositionList), len(eyePositionList)))
                
                eyePositionCalculator.setOriginEyePosition(result)
                originEyePosition = eyePositionCalculator.getOriginEyePosition()

                isMeasuring = False
                measuredTime = 0
                eyePositionList.clear()


        # Show frame
        if SHOW_RESULT:
            cv2.imshow(mainWindowName, frame)

        # Wait
        inputKey = cv2.waitKey(1)
        
        # Key event
        if inputKey == ord('q'):
            break;

        elif inputKey == ord('p'):
            SHOW_RESULT = not SHOW_RESULT

        elif inputKey == ord('s'):
            targetModeState = not boardManager.getSettingMode()
            boardManager.setSettingMode(targetModeState)

            if not targetModeState:
                BOARD_POINTS = boardManager.getBoardEdgePostionArr()
                perspectiveTransformManager.setPoints(BOARD_POINTS)

        elif inputKey == ord('e'):
            isMeasuring = True

        elif inputKey == ord('t'):
            eyePositionCalculator.setOriginEyePosition(eyePositionOnBoard)
            originEyePosition = eyePositionCalculator.getOriginEyePosition()

        elif inputKey == ord('r'):
            boardManager.pause()

                        

os.system(f"v4l2-ctl -d {CAM_INDEX} --set-ctrl exposure_auto_priority=0")

def onRosShutdown():
    Log.d("ROS Shutdown", "SHUTDOWN!")
    cv2.destroyAllWindows()
    sys.exit()
rospy.on_shutdown(onRosShutdown)

multiprocessingManager = multiprocessing.Manager()
eyePosition = multiprocessingManager.list()
eyePosition.append(0)
eyePosition.append(0)

p2 = multiprocessing.Process(target=mainLoop, args=[eyePosition])
p2.start()

if RUN_WITH_ROS:
    p1 = multiprocessing.Process(target=publish, args=[eyePosition])
    p1.start()
    p1.join()
p2.join()

# mainLoop()
