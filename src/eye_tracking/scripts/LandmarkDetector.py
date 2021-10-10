import numpy
import dlib

class LandmarkDetector:
    def __init__(self, predictor, landmarkCount):
        self._predictor = predictor
        self._landmarkCount = landmarkCount

    def detect(self, grayFrame, face):
        (x, y, w, h) = face
        dlibRect = dlib.rectangle(x, y, x + w, y + h)
        landmarks = numpy.matrix([[p.x, p.y] for p in self._predictor(grayFrame, dlibRect).parts()])
        landmarksDisplay = landmarks[0:self._landmarkCount]
        
        points = []
        for idx, point in enumerate(landmarksDisplay):
            pos = numpy.array([
                int(point[0, 0]), int(point[0, 1])
            ])
            points.append(pos)
        
        return points
