#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("test_moveit");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor](){ executor.spin();});
    auto arm = moveit::planning_interface::MoveGroupInterface(node,"arm");
    auto gripper = moveit::planning_interface::MoveGroupInterface(node,"gripper");

    arm.setMaxVelocityScalingFactor(1.0);
    arm.setMaxAccelerationScalingFactor(1.0);
    gripper.setMaxVelocityScalingFactor(1.0);
    gripper.setMaxAccelerationScalingFactor(1.0);
    auto logger = gripper.getNode()->get_logger();


    //Named goal
    // example of publishing a plan.

    // arm.setStartStateToCurrentState();
    // arm.setNamedTarget("home");

    // gripper.setStartStateToCurrentState();
    // gripper.setNamedTarget("open");

    // moveit::planning_interface::MoveGroupInterface::Plan plan9;
    // bool success = (gripper.plan(plan9) == moveit::core::MoveItErrorCode::SUCCESS);
    // if(success)
    // {
    //     //arm.execute(plan9);
    //     gripper.execute(plan9);
    //     RCLCPP_WARN(logger, "gripper success, executing...");

    // }

    // arm.setStartStateToCurrentState();
    // arm.setNamedTarget("pose_1");

    // moveit::planning_interface::MoveGroupInterface::Plan plan1;
    // success = (arm.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);
    // if(success)
    // {
    //     arm.execute(plan1);
    // }
//********************************************************** */

 //********************************************************** */

    //  arm.setStartStateToCurrentState();
    // arm.setNamedTarget("pose_2");

    // moveit::planning_interface::MoveGroupInterface::Plan plan3;
    // success = (arm.plan(plan3) == moveit::core::MoveItErrorCode::SUCCESS);
    // if(success)
    // {
    //     arm.execute(plan3);
    // }
//********************************************************** */
    // gripper.setStartStateToCurrentState();
    // gripper.setNamedTarget("close");

    // moveit::planning_interface::MoveGroupInterface::Plan plan19;
    // success = (gripper.plan(plan19) == moveit::core::MoveItErrorCode::SUCCESS);
    // if(success)
    // {
    //     //arm.execute(plan9);
    //     gripper.execute(plan19);
    //     RCLCPP_WARN(logger, "gripper success, executing...");

    // }
 //********************************************************** */

    // arm.setStartStateToCurrentState();
    // arm.setNamedTarget("pose_3");

    // moveit::planning_interface::MoveGroupInterface::Plan plan5;
    // success = (arm.plan(plan5) == moveit::core::MoveItErrorCode::SUCCESS);
    // if(success)
    // {
    //     arm.execute(plan5);
    // }

    // gripper.setStartStateToCurrentState();
    // gripper.setNamedTarget("open");

    // moveit::planning_interface::MoveGroupInterface::Plan plan29;
    // success = (gripper.plan(plan29) == moveit::core::MoveItErrorCode::SUCCESS);
    // if(success)
    // {
    //     //arm.execute(plan9);
    //     gripper.execute(plan29);
    //     RCLCPP_WARN(logger, "gripper success, executing...");

    // }
    // for(int counter=0;counter<=100000;counter++)
    // {
    //     RCLCPP_INFO(logger,"couting...%d",counter);

    // }
    // gripper.setStartStateToCurrentState();
    // gripper.setNamedTarget("close");

    // moveit::planning_interface::MoveGroupInterface::Plan plan219;
    // success = (gripper.plan(plan219) == moveit::core::MoveItErrorCode::SUCCESS);
    // if(success)
    // {
    //     //arm.execute(plan9);
    //     gripper.execute(plan219);
    //     RCLCPP_WARN(logger, "gripper success, executing...");

    // }

    // for(int counter=0;counter<=500000;counter++)
    // {
    //     RCLCPP_INFO(logger,"couting...%d",counter);

    // }
    // gripper.setStartStateToCurrentState();
    // gripper.setNamedTarget("open");

    // moveit::planning_interface::MoveGroupInterface::Plan plan289;
    // success = (gripper.plan(plan289) == moveit::core::MoveItErrorCode::SUCCESS);
    // if(success)
    // {
    //     //arm.execute(plan9);
    //     gripper.execute(plan289);
    //     RCLCPP_WARN(logger, "gripper success, executing...");

    // }
    // for(int counter=0;counter<=100000;counter++)
    // {
    //     RCLCPP_INFO(logger,"couting...%d",counter);

    // }

    // gripper.setStartStateToCurrentState();
    // gripper.setNamedTarget("close");

    // moveit::planning_interface::MoveGroupInterface::Plan plan239;
    // success = (gripper.plan(plan239) == moveit::core::MoveItErrorCode::SUCCESS);
    // if(success)
    // {
    //     //arm.execute(plan9);
    //     gripper.execute(plan239);
    //     RCLCPP_WARN(logger, "gripper success, executing...");

    // }
     //********************************************************** */

    // arm.setStartStateToCurrentState();
    // arm.setNamedTarget("pose_4");

    // moveit::planning_interface::MoveGroupInterface::Plan plan11;
    // success = (arm.plan(plan11) == moveit::core::MoveItErrorCode::SUCCESS);
    // if(success)
    // {
    //     arm.execute(plan11);
    // }

    // for(int counter=0;counter<=300000;counter++)
    // {
    //     RCLCPP_INFO(logger,"couting...%d",counter);

    // }
//********************************************************** */
    arm.setStartStateToCurrentState();
    arm.setNamedTarget("home");

    moveit::planning_interface::MoveGroupInterface::Plan plan10;
    bool success = (arm.plan(plan10) == moveit::core::MoveItErrorCode::SUCCESS);
    if(success)
    {
        arm.execute(plan10);
    }
 //********************************************************** */

 //Joint Goal
    // std::vector<double> joints = { 1.5, 0.5, 0.0, 1.5, 0.0, -0.7 };

    // arm.setStartStateToCurrentState();
    // arm.setJointValueTarget(joints);

    // moveit::planning_interface::MoveGroupInterface::Plan plan6;
    // success = (arm.plan(plan6) == moveit::core::MoveItErrorCode::SUCCESS);
    // if(success)
    // {
    //     arm.execute(plan6);
    // }

    // tf2::Quaternion q;
    // q.setRPY(0.0,0.0,3.14);
    // q = q.normalize();

    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.frame_id = "base_link";
    target_pose.pose.position.x = 0.3;
    target_pose.pose.position.y = 0.0;
    target_pose.pose.position.z = 0.3;
    // roll=5°、pitch=90°、yaw=0° 对应的四元数
    // roll=5°、pitch=90°、yaw=0° 对应的四元数
    target_pose.pose.orientation.x = 0.3827;
    target_pose.pose.orientation.y = 0.0;
    target_pose.pose.orientation.z = 0.0;
    target_pose.pose.orientation.w = 0.9249;


    arm.setStartStateToCurrentState();
    arm.setPoseTarget(target_pose);
    moveit::planning_interface::MoveGroupInterface::Plan plan7;
    success = (arm.plan(plan7) == moveit::core::MoveItErrorCode::SUCCESS);
    if(success)
    {
        RCLCPP_WARN(logger, "Plan 7 (pose_7) success, executing...");
        arm.execute(plan7);
    }    

  
    // //Cartian Path
    // std::vector<geometry_msgs::msg::Pose> waypoints;
    // geometry_msgs::msg::Pose pose1 = arm.getCurrentPose().pose;
    // pose1.position.z += 0.1;
    // waypoints.push_back(pose1);
    // geometry_msgs::msg::Pose pose2 = pose1;
    // pose2.position.x -= 0.1;
    // waypoints.push_back(pose2);
    // geometry_msgs::msg::Pose pose3 = pose2;
    // pose3.position.x += 0.1;
    // pose3.position.z -= 0.1;
    // waypoints.push_back(pose3);

    // geometry_msgs::msg::Pose pose4 = pose3;
    // pose4.position.y += 0.1;
    // waypoints.push_back(pose4);

    // moveit_msgs::msg::RobotTrajectory trajectory;

    // double fraction = arm.computeCartesianPath(waypoints,0.01,trajectory);
    // arm.execute(trajectory);
    // RCLCPP_INFO(logger, "fraction= %f\n",fraction);
    // // }

    // arm.setStartStateToCurrentState();
    // arm.setNamedTarget("home");

    // moveit::planning_interface::MoveGroupInterface::Plan plan8;
    // success = (arm.plan(plan8) == moveit::core::MoveItErrorCode::SUCCESS);
    // if(success)
    // {
    //     arm.execute(plan8);
    // }


    rclcpp::shutdown();
    spinner.join();
    return 0;

}