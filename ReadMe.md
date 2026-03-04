## 概述
inno_double_wedge_driver_ws 是由苏州智驰领驭开发的 ROS 驱动软件包，专门用于支持 Innolidar 激光雷达。该软件包支持 ROS1 ，提供点云数据处理、IMU GPS数据解析等功能，并包含雷达驱动内核及 ROS1 扩展功能。

## 依赖库安装
根据您的系统选择对应的 ROS 版本：

ROS1:

    Ubuntu 16.04 - ROS kinetic desktop-full
    Ubuntu 18.04 - ROS melodic desktop-full
    Ubuntu 20.04 - ROS noetic desktop-full

安装方式：参考 http://wiki.ros.org

## 编译与安装
### ROS1 编译方式

设置编译方法为 CATKIN
git clone https://github.com/innolidar/inno_double_wedge_driver_ws.git

cd inno_double_wedge_driver
catkin_make

## 激光雷达注意事项
雷达IP：192.168.0.X

电脑IP：192.168.0.3 端口：2368

RK3588 IP：192.168.1.12
