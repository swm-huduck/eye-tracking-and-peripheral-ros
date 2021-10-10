import time

class FpsManager:
    def __init__(self):
        self._prevTime = -1;  # ms
        self._curTime = 0;   # ms
        self._deltaTime = 0; # ms
        self._fps = 0;

    def _getCurMiliTime(self):
        return int(time.time() * 1000.0);

    def updateFps(self):
        if self._prevTime == -1:
            self._prevTime = self._getCurMiliTime()

        self._curTime = self._getCurMiliTime();
        self._deltaTime = self._curTime - self._prevTime
        if self._deltaTime != 0:
            self._fps = int(1000 / self._deltaTime)
        self._prevTime = self._curTime

    def getFps(self):
        return self._fps

    def getDeltaTime(self):
        return self._deltaTime

if __name__ == "__main__":
    fpsManager = FpsManager()
    fpsManager.updateFps()
    print(fpsManager.getFps())
    print(fpsManager.getDeltaTime())
