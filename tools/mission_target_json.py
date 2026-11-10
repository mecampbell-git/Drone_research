import json
import os
import numpy as np
from mission_header import MissionCommand, TargetMode


mount_rpy_frd = [0.0, -1.05, 0.0]
fov = [85.3, 59.9]  # sim pinhole camera (horizontal_fov 1.4888 rad, 1920x1200)
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

# Using NED here just for testing in sim, real world would use lat/lon/alt
wp1 = [0.0, 0.0, -20.0, 10.0]  
wp2 = [110.0,16.0,-20.0, 5.0]
wp3 = [-3.0, 26.0, -20.0, 8.0]
wp4 = [-35.0, -10.0, -20.0, 2.0]
wp5 = [-100.0, 50.0, -20.0, 5.0]


mission_dict = {
    "camera" : {
        "mount_rpy_frd" : mount_rpy_frd,
        "fov" : fov,
        "img_size" : img_size,
        "frame_rate" : frame_rate
    },
    "params" : {
        "wp_format" : "NED",
        "max_speed" : max_speed,
        "acceptance_radius" : acceptance_radius,
        "lookahead" : lookahead,
        "autostart" : True
    },
    "items" : [
        {
            "command" : MissionCommand.WP.value,
            "params" : wp1,
            "flythrough" : True
        },
        {
            "command" : MissionCommand.WP.value,
            "params" : wp2,
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
            "flythrough" : True
        },
        {
            "command" : MissionCommand.WP.value,
            "params" : wp5,
            "flythrough" : False
        },
    ]
}
fpath = "/home/martin/Projects/Drone_research/ws/src/px4_offboard/missions/" 
file = "mission_target.json"
path = os.path.join(fpath, file)

with open(path, "w") as file:
    json.dump(mission_dict, file, indent=4)

with open(path, "r") as file:
    loaded_mission = json.load(file)

wp_indices = []
for i in range(len(loaded_mission["items"])):
    print(f"Item {i}: {loaded_mission['items'][i]}")
    if loaded_mission["items"][i]["command"] == MissionCommand.WP.value:
        wp_indices.append(i)

print(f"WP indices: {wp_indices}")

for i in range(len(wp_indices)):
    index = wp_indices[i]
    wp_params = loaded_mission["items"][index]["params"]
    print(f"Waypoint:{wp_params[0:3]}")
    print(f"velocity:{wp_params[3]}")
