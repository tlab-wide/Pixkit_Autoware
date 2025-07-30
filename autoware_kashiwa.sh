#!/bin/bash
source /home/autoware/pix_autoware/autoware/install/setup.bash 
ros2 launch autoware_launch autoware.launch.xml vehicle_model:=pixkit sensor_model:=pixkit_sensor_kit map_path:=/home/autoware/pix_autoware/autoware_map/kashiwa
