import json
import os
import numpy as np
from mission_header import MissionCommand, TargetMode


mount_rpy_frd = [0.0, -1.05, 0.0]

# Used for targetting in simulation only
# fov = [87.7, 62.0]  # Original values used for sim
fov = [85.3, 59.9]  # sim pinhole camera (horizontal_fov 1.4888 rad, 1920x1200)

"""
# Real camera values from calibration. Not needed for mission execution, maybe for planning(future)
fov1920x1200 = [109.5, 66.8]
fov_other = [109.5, 59.5]  # for 1920x1080 and 1280x720
"""

img_size = [1920, 1200]
frame_rate = 30


max_speed = 10.0
acceptance_radius = 0.5
lookahead = 6.0


"""
# Example commands
        {
            "command" : MissionCommand.RECORD.value,
            "params" : True
        },
        {
            "command" : MissionCommand.TARGET.value,
            "mode" : TargetMode.LOG_REPORT.value,
            "targets" : ["pedestrian", "car"]
        },
        {
            "command" : MissionCommand.TARGET.value,
            "mode" : TargetMode.OFF.value
        },
"""


wp1 = [37.87145913, -122.32004523, 10, 5]  # lat, lon, alt, vel
wp2 = [37.87140113, -122.31924254, 10 , 5]
wp3 = [37.87196616, -122.31897867, 15, 8]
wp4 = [37.87188384, -122.3198651, 20 , 5]

mission_dict = {
    "camera" : {
        "mount_rpy_frd" : mount_rpy_frd,
        "fov" : fov,
        "img_size" : img_size,
        "frame_rate" : frame_rate
    },
    "params" : {
        "wp_format" : "LLA",
        "max_speed" : max_speed,
        "acceptance_radius" : acceptance_radius,
        "lookahead" : lookahead,
        "autostart" : True
    },
    "items" : [
        {
            "command" : MissionCommand.WP.value,
            "params" : wp1,
            "flythrough" : False
        },
        {
            "command" : MissionCommand.SCAN360.value,
            "params" : 30
        },
        {
            "command" : MissionCommand.PAUSE.value,
            "params" : 5
        },
        {
            "command" : MissionCommand.WP.value,
            "params" : wp2,
            "flythrough" : True

        },
        {
            "command" : MissionCommand.WP.value,
            "params" : wp3,
            "flythrough" : False
        },
        {
            "command" : MissionCommand.PAUSE.value,
            "params" : 5
        },
        {
            "command" : MissionCommand.WP.value,
            "params" : wp4,
            "flythrough" : False
        },
    ]
}

fpath = "/home/martin/Projects/Drone_research/ws/src/px4_offboard/missions/" 
file = "marina_box.json"
path = os.path.join(fpath, file)

with open(path, "w") as file:
    json.dump(mission_dict, file, indent=4)

with open(path, "r") as file:
    loaded_mission = json.load(file)

# for item in loaded_mission["items"]:    
#     print(item)
wp_indices = []
for i in range(len(loaded_mission["items"])):
    print(f"Item {i}: {loaded_mission['items'][i]}")
    if loaded_mission["items"][i]["command"] == MissionCommand.WP.value:
        wp_indices.append(i)

print(f"WP indices: {wp_indices}")

loaded_mission["camera"]["mount_rpy_frd"] = np.array(loaded_mission["camera"]["mount_rpy_frd"])
loaded_mission["camera"]["fov"] = np.radians(loaded_mission["camera"]["fov"]) 
loaded_mission["camera"]["img_size"] = tuple(loaded_mission["camera"]["img_size"])

print(f"Camera mount_rpy_frd: {loaded_mission['camera']['mount_rpy_frd']}")
print(f"Camera fov (radians): {loaded_mission['camera']['fov']}")
print(f"Camera img_size: {loaded_mission['camera']['img_size']}")

for i in range(len(wp_indices)):
    index = wp_indices[i]
    wp_params = loaded_mission["items"][index]["params"]
    print(f"Waypoint:{wp_params[0:3]}")
    print(f"velocity:{wp_params[3]}")
