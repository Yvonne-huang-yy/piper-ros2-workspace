测试xacro模型在rviz界面的命令：
ros2 launch urdf_tutorial display.launch.py model:=/home/hanks/ros2_ws/src/my_robot_description/urdf/my_robot.urdf.xacro

测试可视化launch文件命令：
ros2 launch my_robot_description display.launch.xml 


ros2 launch my_robot_bringup start_piper.launch.xml


ros2 launch my_robot_moveit_config demo.launch.py
 ros2 launch my_robot_bringup start_piper.launch.xml
 
启动机械臂
source install/setup.bash
 bash can_activate.sh can0 1000000
 ros2 launch my_robot_bringup my_robot.launch.xml
 ros2 run my_robot_commander_cpp test_moveit
 
失能机械臂
ros2 topic pub /enable_flag std_msgs/msg/Bool data:\ false\ 
 
启动深度相机
with ros2 run:
ros2 run realsense2_camera realsense2_camera_node
# or, with parameters, for example - temporal and spatial filters are enabled:
ros2 run realsense2_camera realsense2_camera_node --ros-args -p enable_color:=false -p spatial_filter.enable:=true -p temporal_filter.enable:=true

with ros2 launch:
ros2 launch realsense2_camera rs_launch.py
ros2 launch realsense2_camera rs_launch.py depth_module.depth_profile:=1280x720x30 pointcloud.enable:=true

启动手柄
ros2 run joy joy_node
启动控制节点
ros2 run my_robot_commander_cpp commander

ros2 topic pub -1 /open_gripper example_interfaces/msg/Bool "{data: true}"
ros2 topic pub -1 /joint_command example_interfaces/msg/Float64MultiArray "{data: [0, 0, 0, 0, 0, 0]}"
ros2 topic pub -1 /pose_command my_robot_interfaces/msg/PoseCommand "{x: 0.3, y: 0.0, z: 0.2, roll: 0.0, pitch: 1.5708, yaw: 0.0, cartesian_path: true}"
ros2 topic pub -1 /pose_command my_robot_interfaces/msg/PoseCommand "{x: 0.2, y: 0.0, z: 0.2, roll: 0.0, pitch: 1.5708, yaw: 0.0, cartesian_path: false}"




