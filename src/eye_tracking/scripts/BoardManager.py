import math
import configparser
import numpy
import cv2

class BoardManager:
    def __init__(self, iniFilePath):
        self._iniFilePath = iniFilePath
        self._config = configparser.ConfigParser()
        
        self._settingMode = False
        self._selectedEdge = None
        
        self._settingModeFramePause = False
        self._pauseFrame = None
        
        self._readIniFile()

    def _readIniFile(self):
        self._config.read(self._iniFilePath)

        position = self._config["board_edge_position"]

        self._top_left =        numpy.fromstring(position["top_left"],      dtype=int, sep=" ")
        self._top_right =       numpy.fromstring(position["top_right"],     dtype=int, sep=" ")
        self._bottom_left =     numpy.fromstring(position["bottom_left"],   dtype=int, sep=" ")
        self._bottom_right =    numpy.fromstring(position["bottom_right"],  dtype=int, sep=" ")

        self._edgePositionArr = (
            self._top_left,
            self._top_right,
            self._bottom_left,
            self._bottom_right,
        )

        boardSize = self._config["board_size"]
        self._boardSize = (int(boardSize["width"]), int(boardSize["height"]))

    def _writeIniFile(self):
        position = self._config["board_edge_position"]

        position["top_left"]        = f"{self._top_left[0]}     {self._top_left[1]}"
        position["top_right"]       = f"{self._top_right[0]}    {self._top_right[1]}"
        position["bottom_left"]     = f"{self._bottom_left[0]}  {self._bottom_left[1]}"
        position["bottom_right"]    = f"{self._bottom_right[0]} {self._bottom_right[1]}"

        with open(self._iniFilePath, 'w') as  configfile:
            self._config.write(configfile)

    def getBoardEdgePostionArr(self):
        return numpy.array(self._edgePositionArr)

    def setSettingMode(self, bool):
        self._settingMode = bool
        if bool:
            cv2.namedWindow("board_setting", cv2.WINDOW_NORMAL)
            cv2.setMouseCallback("board_setting", self._mouseCallback)
            
        else:
            cv2.destroyWindow("board_setting")
            self._pauseFrame = None
            self._settingModeFramePause = False

    def getSettingMode(self):
        return self._settingMode

    def getBoardSize(self):
        return self._boardSize

    def drawBoardEnge(self, frame):
        if not self._settingMode:
            return
        
        self._frame = frame.copy()

        if self._settingModeFramePause:
            frame = self._pauseFrame.copy()
        else:
            frame = frame.copy()

        cv2.line(frame, tuple(self._top_left),      tuple(self._top_right),     (0, 0, 255))
        cv2.line(frame, tuple(self._top_right),     tuple(self._bottom_right),  (0, 0, 255))
        cv2.line(frame, tuple(self._bottom_right),  tuple(self._bottom_left),   (0, 0, 255))
        cv2.line(frame, tuple(self._bottom_left),   tuple(self._top_left),      (0, 0, 255))

        for egde in self._edgePositionArr:
            cv2.circle(frame, tuple(egde), 2, (0, 0, 255))

        cv2.imshow("board_setting", frame)

    def pause(self):
        if not self.getSettingMode():
            return

        self._settingModeFramePause = not self._settingModeFramePause
        if self._settingModeFramePause:
            self._pauseFrame = self._frame

    def _mouseCallback(self, event, x, y, flags, userdata):
        if not self._settingMode:
            return
        
        if event == cv2.EVENT_LBUTTONDOWN:
            for edge in self._edgePositionArr:
                delX = edge[0] - x
                delY = edge[1] - y
                distance = math.sqrt((delX **2) + (delY ** 2))
                if distance < 10:
                    self._selectedEdge = edge
                    break

        elif event == cv2.EVENT_LBUTTONUP:
            self._selectedEdge = None
            self._writeIniFile()

        elif event == cv2.EVENT_MOUSEMOVE and self._selectedEdge is not None:
            self._selectedEdge[0] = x
            self._selectedEdge[1] = y
            
        