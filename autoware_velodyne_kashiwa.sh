#!/bin/bash
source /home/autoware/pixkit_autoware_0.45.1/autoware/install/setup.bash 
ros2 launch autoware_launch autoware.launch.xml vehicle_model:=pixkit sensor_model:=velodyne_pixkit_sensor_kit map_path:=/home/autoware/pixkit_autoware_0.45.1/autoware_map/kashiwa log_level:=debug
