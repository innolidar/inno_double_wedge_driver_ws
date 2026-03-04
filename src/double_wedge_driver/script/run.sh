#!/bin/bash

# 启动 lidar
gnome-terminal -- bash -c "source devel/setup.bash; roslaunch jp_device_driver start_lidar.launch; exec bash"

sleep 1
# 启动 test_lidar.launch 并开启rviz
gnome-terminal -- bash -c "source devel/setup.bash; roslaunch jp_device_driver test_lidar.launch use_rviz:=true; exec bash"
