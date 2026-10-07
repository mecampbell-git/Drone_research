ros2 launch px4_bringup minimal.launch.py world:=baylands
ros2 launch px4_offboard keyboard_teleop.launch.py

ros2 run px4_offboard position_offboard_control.py
ros2 topic info /fmu/out/vehicle_global_position

ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=bridge_config.yaml
ros2 run rqt_image_view rqt_image_view

# PX4 commands
commander land
commander mode auto:rtl

# Location of drone models. my_x500 and my_x500_base
/home/martin/PX4-Autopilot/Tools/simulation/gz/models/my_x500_base
