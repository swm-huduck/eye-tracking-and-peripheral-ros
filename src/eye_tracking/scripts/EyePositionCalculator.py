import configparser
import cv2
import numpy

class EyePositionCalculator:
    def __init__(self, frameWidth, frameHeight, boardWidth, boardHeight, eyePositionIndexes, iniFilePath):
        self._frameWidth = frameWidth
        self._frameHeight = frameHeight
        self._boardWidth = boardWidth
        self._boardHeight = boardHeight
        self._boardPerFrame = boardWidth / frameWidth
        self._eyePositionIndexes = eyePositionIndexes

        self._config = configparser.ConfigParser()
        self._iniFilePath = iniFilePath;
        self._readIniFile()

    def _readIniFile(self):
        self._config.read(self._iniFilePath)

        origin_eye_position = self._config["origin_eye_position"]
        self._origin_eye_position_x = int(origin_eye_position["x"])
        self._origin_eye_position_y = int(origin_eye_position["y"])

    def _writeIniFile(self):
        origin_eye_position = self._config["origin_eye_position"]
        origin_eye_position["x"] = str(self._origin_eye_position_x)
        origin_eye_position["y"] = str(self._origin_eye_position_y)

        with open(self._iniFilePath, 'w') as  configfile:
            self._config.write(configfile)

    def getOriginEyePosition(self):
        return numpy.array((self._origin_eye_position_x, self._origin_eye_position_y))


    def setOriginEyePosition(self, originEyePosition):
        if int(originEyePosition[0]) == 0 and int(originEyePosition[1]) == 0:
            return

        origin_eye_position = self._config["origin_eye_position"]
        self._origin_eye_position_x = int(round(originEyePosition[0], 0))
        self._origin_eye_position_y = int(round(originEyePosition[1], 0))

        self._writeIniFile()

    def _drawEye(self, pos1, pos2, center, frame):
        pos1 = tuple(pos1.astype(int))
        pos2 = tuple(pos2.astype(int))
        center = tuple(center.astype(int))

        cv2.circle(frame, pos1, 2, (0, 255, 0), 2)
        cv2.circle(frame, pos2, 2, (0, 255, 0), 2)
        cv2.circle(frame, center, 4, (0, 255, 0), 2)

    def _drawEyeCenter(self, center, distance, frame):
        center = tuple(center.astype(int))
        virPos = (center[0], 0)
        horPos = (0, center[1])

        cv2.line(frame, center, virPos, (0, 0, 255), 2)
        cv2.line(frame, center, horPos, (0, 0, 255), 2)

        virDist = round(distance[1] / 10, 1)
        horDist = round(distance[0] / 10, 1)

        cv2.putText(frame, f"{virDist}cm", (virPos[0] + 30, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0, 255), 2)
        cv2.putText(frame, f"{horDist}cm", (30, horPos[1] + 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0, 255), 2)

    def calc(self, points, debugFrame=None):
        leftEyeOutSidePos = points[self._eyePositionIndexes[0]]
        leftEyeInSidePos = points[self._eyePositionIndexes[1]]
        leftEyeCenterPos = (leftEyeOutSidePos + leftEyeInSidePos) / 2

        rightEyeInSidePos = points[self._eyePositionIndexes[2]]
        rightEyeOutSidePos = points[self._eyePositionIndexes[3]]
        rightEyeCenterPos = (rightEyeInSidePos + rightEyeOutSidePos) / 2

        eyeCenterPos = (leftEyeCenterPos + rightEyeCenterPos) / 2
        
        result = eyeCenterPos * self._boardPerFrame
        result = result.astype(int)

        if debugFrame is not None:
            self._drawEye(leftEyeOutSidePos, leftEyeInSidePos, leftEyeCenterPos, debugFrame)
            self._drawEye(rightEyeOutSidePos, rightEyeInSidePos, rightEyeCenterPos, debugFrame)
            self._drawEyeCenter(eyeCenterPos, result, debugFrame)

        return result