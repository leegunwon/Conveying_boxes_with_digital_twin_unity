#include <memory>
#include <vector>
#include <cmath>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>

#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Geometry>  
#include <vector>
#include <tuple>

#define PI 3.14159265358979323846



geometry_msgs::msg::Quaternion rpyToQuaternion(double roll, double pitch, double yaw) {
  Eigen::Quaterniond q;
  q = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
    * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
    * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());

  geometry_msgs::msg::Quaternion quaternion;
  quaternion.w = q.w();
  quaternion.x = q.x();
  quaternion.y = q.y();
  quaternion.z = q.z();
  return quaternion;
}

bool planAndExecute(moveit::planning_interface::MoveGroupInterface& move_group_interface, moveit::planning_interface::MoveGroupInterface::Plan& plan, rclcpp::Logger logger) {
    bool success = static_cast<bool>(move_group_interface.plan(plan));
    if (success) {
        move_group_interface.execute(plan);
        RCLCPP_INFO(logger, "Execution successful!");
        return true;
    } else {
        RCLCPP_ERROR(logger, "Planning failed!");
        return false;
    }
}

geometry_msgs::msg::Quaternion multiply(const geometry_msgs::msg::Quaternion& q2, const geometry_msgs::msg::Quaternion& q1) {
    geometry_msgs::msg::Quaternion out;
    out.w = q2.w*q1.w - q2.x*q1.x - q2.y*q1.y - q2.z*q1.z;
    out.x = q2.w*q1.x + q2.x*q1.w + q2.y*q1.z - q2.z*q1.y;
    out.y = q2.w*q1.y - q2.x*q1.z + q2.y*q1.w + q2.z*q1.x;
    out.z = q2.w*q1.z + q2.x*q1.y - q2.y*q1.x + q2.z*q1.w;
    return out;
}

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "turtlebot_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner = std::thread([&executor]() { executor.spin(); });

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("turtlebot_moveit");
  RCLCPP_INFO(logger, "Hello, MoveIt!"); // Use RCLCPP_INFO for logging

  // moveit core 의 arm 그룹을 사용하기위한 선언
  using moveit::planning_interface::MoveGroupInterface;
  auto arm_interface = MoveGroupInterface(node, "arm");
  arm_interface.setPlanningPipelineId("move_group");
  arm_interface.setPlanningTime(10.0); 

  // moveit core 의 gripper 그룹을 사용하기위한 선언
  auto gripper_interface = MoveGroupInterface(node, "gripper");

  // 로봇 상태를 가져오기 위해 로봇 상태 모니터를 설정
  auto robot_state = arm_interface.getCurrentState(10.0);
  if (!robot_state) {
    RCLCPP_ERROR(logger, "Failed to fetch current robot state");
    rclcpp::shutdown();
    return 1;
  }
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  arm_interface.setGoalPositionTolerance(0.01);
  arm_interface.setGoalOrientationTolerance(0.05);
  
  geometry_msgs::msg::Pose current_pose = arm_interface.getCurrentPose().pose;
  RCLCPP_INFO(node->get_logger(), "Current pose: x: %f y: %f z: %f ox: %f oy: %f oz: %f ow: %f",
    current_pose.position.x,
    current_pose.position.y,
    current_pose.position.z,
    current_pose.orientation.x,
    current_pose.orientation.y,
    current_pose.orientation.z,
    current_pose.orientation.w
  );
  



  // //   // Joint 목표를 를 통한 이동.
  // std::vector<double> joint_group_positions;
  // arm_interface.getCurrentState()->copyJointGroupPositions(arm_interface.getCurrentState()->getRobotModel()->getJointModelGroup(arm_interface.getName()), joint_group_positions);

  //   // 조인트 목표를 설정합니다. (예: 4개의 조인트를 가진 로봇)
  //   joint_group_positions[0] = 0.0;  // joint1
  //   joint_group_positions[1] = 0.0;  // joint2
  //   joint_group_positions[2] = 0.0;  // joint3
  //   joint_group_positions[3] = 0.0;  // joint4

  //   arm_interface.setJointValueTarget(joint_group_positions);

  // 좌표를 통한 moveit 이동
  // x: 0.054273 y: 0.081886 z: 0.230911 ox: -0.188549 oy: 0.671301 oz: 0.193828 ow: 0.690098
  float target_x = 0.054273;
  float target_y = 0.081886;
  float target_z = 0.23;
  // float target_ox = -0.188549;
  // float target_oy = 0.671301;
  // float target_oz = 0.193828;
  // float target_ow = 0.690098;


  //x: 0.104811 y: -0.000283 z: 0.195225 ox: 0.000530 oy: 0.690651 oz: -0.000555 ow: 0.723188
  // float target_x = 0.104811;
  // float target_y = -0.000283;
  // float target_z = 0.195225;


  double yaw = atan2(target_y, target_x+0.08);  // Example yaw value using atan2
  // yaw = (yaw<0)? -PI/2 - yaw : PI/2+yaw; 
  std::cout << "yaw: " << yaw << std::endl;
  geometry_msgs::msg::Quaternion test_q = rpyToQuaternion(0, PI/2, 0); 
  geometry_msgs::msg::Quaternion quaternion = rpyToQuaternion(0, 0, yaw); 
  std::cout << "quaternion: " << quaternion.x << " " << quaternion.y << " " << quaternion.z << " " << quaternion.w << std::endl;

  // geometry_msgs::msg::Quaternion multiply_quaternion = multiply(quaternion,current_pose.orientation);

  geometry_msgs::msg::Quaternion multiply_quaternion = multiply(rpyToQuaternion(0, 0, yaw),rpyToQuaternion(0, PI/2, 0));
  std::cout << "multiply_quaternion: " << multiply_quaternion.x << " " << multiply_quaternion.y << " " << multiply_quaternion.z << " " << multiply_quaternion.w << std::endl;

  current_pose = arm_interface.getCurrentPose().pose;
  RCLCPP_INFO(node->get_logger(), "Current pose: x: %f y: %f z: %f ox: %f oy: %f oz: %f ow: %f",
    current_pose.position.x,
    current_pose.position.y,
    current_pose.position.z,
    current_pose.orientation.x,
    current_pose.orientation.y,
    current_pose.orientation.z,
    current_pose.orientation.w
  );

  auto const target_pose = [test_q,current_pose,multiply_quaternion, target_x, target_y, target_z]{
    geometry_msgs::msg::Pose msg;
    msg.orientation = multiply_quaternion;
    msg.position.x = target_x;
    msg.position.y = target_y;
    msg.position.z = target_z; //-> z값을 -3cm
    return msg;
  }();
  arm_interface.setPoseTarget(target_pose);
  planAndExecute(arm_interface, plan, logger);

  current_pose = arm_interface.getCurrentPose().pose;
  RCLCPP_INFO(node->get_logger(), "Current pose: x: %f y: %f z: %f ox: %f oy: %f oz: %f ow: %f",
    current_pose.position.x,
    current_pose.position.y,
    current_pose.position.z,
    current_pose.orientation.x,
    current_pose.orientation.y,
    current_pose.orientation.z,
    current_pose.orientation.w
  );


  // auto const target_pose2 = [quaternion, target_x, target_y, target_z, target_ox, target_oy, target_oz, target_ow]{
  //   geometry_msgs::msg::Pose msg;
  //   msg.orientation.w = quaternion.w;
  //   msg.orientation.x = quaternion.x;
  //   msg.orientation.y = quaternion.y;
  //   msg.orientation.z = quaternion.z;
  //   msg.position.x = target_x;
  //   msg.position.y = target_y;
  //   msg.position.z = target_z; 
  //   return msg;
  // }();
  // arm_interface.setPoseTarget(target_pose2);
  // planAndExecute(arm_interface, plan, logger);


  // current_pose = arm_interface.getCurrentPose().pose;
  // RCLCPP_INFO(node->get_logger(), "Current pose: x: %f y: %f z: %f ox: %f oy: %f oz: %f ow: %f",
  //   current_pose.position.x,
  //   current_pose.position.y,
  //   current_pose.position.z,
  //   current_pose.orientation.x,
  //   current_pose.orientation.y,
  //   current_pose.orientation.z,
  //   current_pose.orientation.w
  // );
  
  // auto const target_pose = [current_pose]{
  //   geometry_msgs::msg::Pose msg;
  //   msg.orientation.w = current_pose.orientation.w;
  //   msg.orientation.x = current_pose.orientation.x;
  //   msg.orientation.y = current_pose.orientation.y;
  //   msg.orientation.z = current_pose.orientation.z;
  //   msg.position.x = current_pose.position.x+0.05;
  //   msg.position.y = current_pose.position.y;
  //   msg.position.z = current_pose.position.z; //-> z값을 -3cm
  //   return msg;
  // }();
  // arm_interface.setPoseTarget(target_pose);
  // planAndExecute(arm_interface, plan, logger);
  // Plan 을 생성
  // auto const [success, plan] = [&arm_interface]{
  //   moveit::planning_interface::MoveGroupInterface::Plan msg;
  //   auto const ok = static_cast<bool>(arm_interface.plan(msg));
  //   return std::make_pair(ok, msg);
  // }();

  // // plan 이 생성되면 execute
  // if(success) {
  //   RCLCPP_INFO(logger, "Planning Success!, execute the plan");
  //   arm_interface.execute(plan);
  // } else {
  //   RCLCPP_ERROR(logger, "Planning failed!");
  // }



  // // Gripper 를 조작 "open"
  // gripper_interface.setNamedTarget("open");
  // gripper_interface.move();

  // //gripper 를 조작 "close"
  // gripper_interface.setNamedTarget("close");
  // gripper_interface.move();

  // //Name Target 을 통한 arm 이동
  // //Name Target 은 turtlebot3_moveit_config/config/turtlebot3.srdf 에 정의되어 있음
  // // <group_state name="home" group="arm">
  // //       <joint name="joint1" value="0"/>
  // //       <joint name="joint2" value="-1"/>
  // //       <joint name="joint3" value="0.7"/>
  // //       <joint name="joint4" value="0.3"/>
  // //   </group_state>
  // // group_state 의 name 을 을 변경하고 joint state 를 추가하여 사용 가능
  // // group="gripper" 로 설정하면 gripper 도 사용 가능
  
  // arm_interface.setNamedTarget("home");
  // arm_interface.move();

  // Shutdown ROS
  rclcpp::shutdown();
  spinner.join();
  return 0;
}