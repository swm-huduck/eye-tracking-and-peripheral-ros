import cv2
import numpy
from Log import Log


class PerspectiveTransformManager:
    TAG = "PerspectiveTransformManager"


    def __init__(self, points, targetFrameWidth, targetFrameHeight):
        self._init = False

        if len(points) < 4:
            Log.d(PerspectiveTransformManager.TAG, "Initialization failed");
            return;

        pts1 = numpy.float32([list(points[0]), list(points[1]), list(points[2]), list(points[3])])
        pts2 = numpy.float32([[0, 0], [targetFrameWidth, 0], [0, targetFrameHeight], [targetFrameWidth, targetFrameHeight]])
        self._M = cv2.getPerspectiveTransform(pts1, pts2)
        self._dsize = (targetFrameWidth, targetFrameHeight)
        self._init = True
        Log.d(PerspectiveTransformManager.TAG, "Initialization successful");


    def perspectiveTransfrom(self, frame):
        if self._init is False:
            return frame

        frame = cv2.warpPerspective(frame, self._M, self._dsize)
        return frame
