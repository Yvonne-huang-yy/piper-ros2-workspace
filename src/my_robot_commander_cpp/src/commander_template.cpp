#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <example_interfaces/msg/bool.hpp>
#include <example_interfaces/msg/float64_multi_array.hpp>
#include <my_robot_interfaces/msg/pose_command.hpp>
#include "geometry_msgs/msg/pose_array.hpp"
#include <Eigen/Geometry>
#include <cmath>

// 新增：TF2 相关头文件（坐标转换核心）
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using Bool = example_interfaces::msg::Bool;
using FloatArray = example_interfaces::msg::Float64MultiArray;
using PoseCmd = my_robot_interfaces::msg::PoseCommand;
using namespace std::placeholders;

class Commander
{
public:
    Commander(std::shared_ptr<rclcpp::Node> node)
    {
        node_ = node;
        arm_ = std::make_shared<MoveGroupInterface>(node_,"arm");
        arm_->setMaxVelocityScalingFactor(1);
        arm_->setMaxAccelerationScalingFactor(1);
        arm_->setPlannerId("RRTConnectkConfigDefault");  // 更快的规划器
        arm_->setPlanningTime(1.0);  // 规划超时时间1秒（默认5秒）
        arm_->allowReplanning(false);  // 关闭重规划（减少开销）

        gripper_ = std::make_shared<MoveGroupInterface>(node_,"gripper");
        open_gripper_sub_ = node_->create_subscription<Bool>(
            "open_gripper",10,std::bind(&Commander::openGripperCallback,this,_1));
        joint_cmd_sub_ = node_->create_subscription<FloatArray>(
            "joint_command",10,std::bind(&Commander::jointCmdCallback,this,_1));
        pose_cmd_sub_ = node->create_subscription<PoseCmd>(
            "pose_command",10,std::bind(&Commander::poseCmdCallback,this,_1));
        aruco_pose_cmd_sub_ = node->create_subscription<geometry_msgs::msg::PoseArray>(
            "/aruco/poses",10,std::bind(&Commander::arucoposeCallback,this,_1));

        // ******************** 新增：初始化 TF 缓冲区和监听器（类内成员）********************
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // ******************** 新增：初始化机械臂当前目标姿态（解决循环震荡）********************
        current_arm_pitch_target_ = 1.5708;  // 初始俯仰角（对应90°）
        decay_factor_ = 0.1;                // 姿态衰减系数（平滑微调）
        pitch_threshold_ = M_PI / 36.0;     // 姿态偏差阈值（±5°，弧度制）
    }

    void goToNamedTarget(const std::string &name)
    {
        arm_->setStartStateToCurrentState();
        arm_->setNamedTarget(name);
        planAndExecute(arm_);
    }

    void goToJointTarget(const std::vector<double> &joints)
    {
        arm_->setStartStateToCurrentState();
        arm_->setJointValueTarget(joints);
        planAndExecute(arm_);
    }

    void goToPoseTarget(double x,double y,double z,double roll, double pitch, double yaw,bool cartesian_path)
    {
        tf2::Quaternion q;
        q.setRPY(roll,pitch,yaw);
        q = q.normalize();

        geometry_msgs::msg::PoseStamped target_pose;
        target_pose.header.frame_id = "base_link";
        target_pose.pose.position.x = x;
        target_pose.pose.position.y = y;
        target_pose.pose.position.z = z;
        target_pose.pose.orientation.x = q.getX();
        target_pose.pose.orientation.y = q.getY();
        target_pose.pose.orientation.z = q.getZ();
        target_pose.pose.orientation.w = q.getW();

        arm_->setStartStateToCurrentState();
        if(!cartesian_path)
        {
            arm_->setPoseTarget(target_pose);
            planAndExecute(arm_);
        }
        else
        {
            std::vector<geometry_msgs::msg::Pose> waypoints;
            waypoints.push_back(target_pose.pose);
            moveit_msgs::msg::RobotTrajectory trajectory;

            double fraction = arm_->computeCartesianPath(waypoints,0.05,trajectory);
            arm_->execute(trajectory);
        }
    }

    void openGripper()
    {
        gripper_->setStartStateToCurrentState();
        gripper_->setNamedTarget("open");
        planAndExecute(gripper_);
    }

    void closeGripper()
    {
        gripper_->setStartStateToCurrentState();
        gripper_->setNamedTarget("close");
        planAndExecute(gripper_);
    }

    void PS5_Control(float AXIS_LEFT_X,float AXIS_LEFT_Y,float TRIGGER_LEFT,
                     float AXIS_RIGHT_X,float AXIS_RIGHT_Y,float TRIGGER_RIGHT,
                     float BOTTON_LEFT_X,float BOTTON_LEFT_Y)
    {
        auto logger = arm_->getNode()->get_logger();
        float joint[6] = {0.0f};
        if(TRIGGER_RIGHT == -1)
        {
            goToJointTarget(std::vector<double>(joint, joint + 6)); 
        }
        else
        {
            // 预留笛卡尔路径控制，保持原有逻辑
        }
        if(TRIGGER_LEFT == -1)
        {
            openGripper();
        }
        else
        {
            closeGripper();
        }
    }

private:
    int timer = 0;

    // ******************** 新增：类内成员变量（TF + 姿态平滑控制）********************
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    double current_arm_pitch_target_;  // 机械臂当前目标俯仰角
    double decay_factor_;              // 姿态衰减系数
    double pitch_threshold_;           // 姿态偏差阈值

    void planAndExecute(const std::shared_ptr<MoveGroupInterface> &interface)
    {
        MoveGroupInterface::Plan plan;
        bool success = (interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if(success)
        {
            // interface->asyncExecute(plan);
            interface->execute(plan);
        }
    }

    void openGripperCallback(const Bool &msg)
    {
        if(msg.data){
            openGripper();
        }
        else{
            closeGripper();
        }
    }

    void jointCmdCallback(const FloatArray &msg)
    {
        auto joints = msg.data;
        if(joints.size() == 6){
            goToJointTarget(joints);
        }
    }

    void poseCmdCallback(const PoseCmd &msg)
    {
        // 预留姿态指令回调，保持原有逻辑
    }

    double normalizeAngleDeg(double angle_deg)
    {
        angle_deg = fmod(angle_deg, 360.0);
        if (angle_deg > 180.0)
        {
            angle_deg -= 360.0;
        }
        else if (angle_deg < -180.0)
        {
            angle_deg += 360.0;
        }
        return angle_deg;
    }

    void arucoposeCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        if (msg->poses.empty())
        {
            RCLCPP_WARN(rclcpp::get_logger("aruco_tracker"), "未检测到ArUco码，poses数组为空！");
            goToPoseTarget(0.3,0.0,0.3,0.0,0.0,0.0,false);
            return;
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger("aruco_tracker"), "tracking...");

            // 1. 提取 ArUco 在 color_optical_frame 下的局部位置和四元数
            double aruco_x_local = msg->poses[0].position.x;
            double aruco_y_local = msg->poses[0].position.y;
            double aruco_z_local = msg->poses[0].position.z;
            double aruco_qx = msg->poses[0].orientation.x;
            double aruco_qy = msg->poses[0].orientation.y;
            double aruco_qz = msg->poses[0].orientation.z;
            double aruco_qw = msg->poses[0].orientation.w;

            // ******************** 新增：ArUco 位置转换到 base_link 坐标系 ********************
            geometry_msgs::msg::PointStamped aruco_point_local;
            geometry_msgs::msg::PointStamped aruco_point_base;

            // 填充局部点信息
            aruco_point_local.header.frame_id = "camera_color_optical_frame";
            aruco_point_local.header.stamp = msg->header.stamp;  // 复用消息时间戳，提高同步性
            aruco_point_local.point.x = aruco_x_local;
            aruco_point_local.point.y = aruco_y_local;
            aruco_point_local.point.z = aruco_z_local;

            try
            {
                // 核心：查询 TF 变换并完成坐标转换
                tf_buffer_->transform(aruco_point_local, aruco_point_base, "base_link", tf2::durationFromSec(0.1));
            }
            catch (tf2::TransformException &ex)
            {
                RCLCPP_ERROR(rclcpp::get_logger("aruco_tracker"), "坐标转换失败：%s", ex.what());
                return;
            }

            // 提取转换后的全局位置（base_link 下）
            double aruco_x_base = aruco_point_base.point.x;
            double aruco_y_base = aruco_point_base.point.y;
            double aruco_z_base = aruco_point_base.point.z;

            // 2. 四元数→Eigen四元数→旋转矩阵
            Eigen::Quaterniond q_aruco(aruco_qw, aruco_qx, aruco_qy, aruco_qz);
            Eigen::Matrix3d R_aruco = q_aruco.toRotationMatrix();

            // 3. 提取自定义轴旋转角度
            double custom_pitch = atan2(R_aruco(2, 1), sqrt(pow(R_aruco(0, 1), 2) + pow(R_aruco(1, 1), 2)));
            double custom_yaw = atan2(R_aruco(0, 2), R_aruco(0, 0));
            double custom_roll = atan2(R_aruco(1, 0), sqrt(pow(R_aruco(0, 0), 2) + pow(R_aruco(2, 0), 2)));
            if(aruco_x_base-0.3>=0.05)
            {
                goToPoseTarget(aruco_x_base-0.3, aruco_y_base, aruco_z_base-0.05,custom_roll, 0.0, 0.0, false);
            }            
            if(aruco_z_local<=0.12)
            {
                openGripper();
            }
            else
            {
                closeGripper();
            }

            // RCLCPP_INFO(rclcpp::get_logger("aruco_tracker"), 
            //             "ArUco 全局位置（base_link）：x=%.3f, y=%.3f, z=%.3f", 
            //             aruco_x_base, aruco_y_base, aruco_z_base);
        }
    }

    // 类原有成员变量
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<MoveGroupInterface> arm_;
    std::shared_ptr<MoveGroupInterface> gripper_;
    rclcpp::Subscription<Bool>::SharedPtr open_gripper_sub_;
    rclcpp::Subscription<FloatArray>::SharedPtr joint_cmd_sub_;
    rclcpp::Subscription<PoseCmd>::SharedPtr pose_cmd_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr aruco_pose_cmd_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<rclcpp::Node>("commander");
    auto commander = Commander(node);
    
    executor.add_node(node);
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}
