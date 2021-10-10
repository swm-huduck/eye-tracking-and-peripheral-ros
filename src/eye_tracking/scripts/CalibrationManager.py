import cv2
import numpy
import glob
from Log import Log


class CalibrationManager:
    TAG = "CalibrationManager"


    def __init__(self, imagePaths, frameWidth, frameHeight):
        self._init = False

        vir = 5
        hor = 7
        criteria = (cv2.TermCriteria_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        objp = numpy.zeros((hor * vir, 3), numpy.float32)
        objp[:, :2] = numpy.mgrid[0:vir, 0:hor].T.reshape(-1, 2)

        objPoints = []
        imgPoints = []

        for imagePath in imagePaths:
            image = cv2.imread(imagePath)
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

            ret, corners = cv2.findChessboardCorners(gray, (vir, hor), None)

            if ret is False:
                continue
            
            if self._init is False:
                self._init = True

            objPoints.append(objp)
            corner2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgPoints.append(corner2)

        if self._init is True:
            Log.d(CalibrationManager.TAG, "Initialization successful");
        else:
            Log.d(CalibrationManager.TAG, "Initialization failed");
            return;

        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objPoints, imgPoints, gray.shape[::-1], None, None)

        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(
            mtx, dist, (frameWidth, frameHeight), 1, (frameWidth, frameHeight)
        )
        self._roiX, self._roiY, self._roiW, self._roiH = roi

        self._mapX, self._mapY = cv2.initUndistortRectifyMap(
            mtx, dist, None, newcameramtx, (frameWidth, frameHeight), 5
        )


    def calibration(self, frame):
        if self._init is False:
            return frame

        dst = cv2.remap(frame, self._mapX, self._mapY, cv2.INTER_LINEAR)
        frame = dst[self._roiY : self._roiY + self._roiH, self._roiX : self._roiX + self._roiW]
        return frame
