from enum import Enum

class MissionCommand(Enum):
    WP = 1
    RECORD = 2
    SCAN360 = 3
    PAUSE = 4
    TARGET = 5
    LAND = 6
    RTL = 7

class TargetMode(Enum):
    OFF = 0
    LOG_REPORT = 1
    PAUSE_WAIT = 2
    MOVE_TO_TARGET = 3
