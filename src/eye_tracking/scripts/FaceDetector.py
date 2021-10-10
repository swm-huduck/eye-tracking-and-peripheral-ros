import cv2

class FaceDetector:
    def __init__(self, cascade, resizeScale):
        self._cascade = cascade
        self._resizeScale = resizeScale
        minSize = int(self._resizeScale * 100)
        self._minSize = (minSize, minSize)


    def detect(self, grayFrame):
        faces = self._cascade.detectMultiScale(grayFrame, scaleFactor=1.05, minNeighbors=5, minSize=self._minSize,
                                                flags=cv2.CASCADE_SCALE_IMAGE)
        
        return faces
        