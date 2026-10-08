# from enum import Enum, auto

# class MissionState(Enum):
#     WARMUP = auto()
#     TRAVEL = auto()
#     WAIT = auto()
#     RTL = auto()

# mission = []
# mission.append([100, 100, -30, 5]) #NED + delay at endpoint
# mission.append([50, 150, -40, 0]) 
# mission.append([150, 100, -60, 10]) 
# mission.append([0, 150, -20, 2]) 
# state = MissionState.WARMUP

import csv

with open("mission1.csv", mode="r", newline="", encoding="utf-8") as f:
    reader = csv.reader(f)
    mission = [[float(cell) for cell in row] for row in reader]

print(mission)

# print(len(mission))

# posx, posy, takeoff_height, delay = mission[0]

# print(f"Next waypoint: {posx}, {posy}, {takeoff_height}, delay: {delay}")