#!/bin/bash

# 1. 启动第一个launch文件，打开新终端
gnome-terminal --title="my_robot_bringup" -- bash -c "ros2 launch my_robot_bringup my_robot.launch.xml; exec bash"

# 延时2秒，确保第一个节点启动完成（可根据实际情况调整）
sleep 1

# 2. 运行commander节点，打开新终端
gnome-terminal --title="my_robot_commander" -- bash -c "ros2 run my_robot_commander_cpp commander; exec bash"

# 再延时2秒
sleep 1

# 3. 启动第三个launch文件，打开新终端
gnome-terminal --title="aruco_pose_estimation" -- bash -c "ros2 launch aruco_pose_estimation aruco_pose_estimation_without_rviz.launch.py; exec bash"

