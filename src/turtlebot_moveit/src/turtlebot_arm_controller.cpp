#include <rclcpp/rclcpp.hpp>
#include <turtlebot_cosmo_interface/srv/moveit_control.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_array.hpp>

#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Geometry>  
#include <vector>
#include <tuple>

#define NEW  0

#define PI 3.14159265358979323846
class TurtlebotArmController : public rclcpp::Node {
public:
    TurtlebotArmController(const rclcpp::NodeOptions &options) : Node("turtlebot_arm_controller",options),
    node_(std::make_shared<rclcpp::Node>("move_group_interface")),           // Create an additional ROS node
    move_group_interface_(node_, "arm"),                                     // Initialize MoveGroupInterface for controlling the arm
    executor_(std::make_shared<rclcpp::executors::SingleThreadedExecutor>()) // Create a single-threaded executor
    {
        service_ = this->create_service<turtlebot_cosmo_interface::srv::MoveitControl>(
            "moveit_control", std::bind(&TurtlebotArmController::handleMoveitControl, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "Ready to receive MoveitControl commands.");
        executor_->add_node(node_);
        executor_thread_ = std::thread([this]() {
            RCLCPP_INFO(node_->get_logger(), "Starting executor thread");    // Log message indicating the thread start
            executor_->spin();                                               // Run the executor to process callbacks
        });
    }

    // 추가
    ~TurtlebotArmController() {
        if (executor_) {
            executor_->cancel();
        }
        if (executor_thread_.joinable()) {
            executor_thread_.join();
        }
    }

    void printCurrentPose() {
        auto current_pose = move_group_interface_.getCurrentPose().pose;     // Get the current pose
        std::cout << "Current Pose:" << std::endl;

        // 쿼터니언 가져오기
        const auto& orientation = current_pose.orientation;

        // tf2::Quaternion 객체로 변환
        tf2::Quaternion quaternion(orientation.x, orientation.y, orientation.z, orientation.w);

        // RPY 계산
        double roll, pitch, yaw;
        tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);

        // RPY 출력
        std::cout << "Roll: " << roll << ", Pitch: " << pitch << ", Yaw: " << yaw << std::endl;

        std::cout<<"yaw : "<<yaw<< std::endl;  
        std::cout << "Position: (" << current_pose.position.x << ", "
                << current_pose.position.y << ", "
                << current_pose.position.z << ")" << std::endl;
        std::cout << "Orientation: (" << current_pose.orientation.x << ", "
                << current_pose.orientation.y << ", "
                << current_pose.orientation.z << ", "
                << current_pose.orientation.w << ")" << std::endl;
    }

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
#if NEW
// 수정
    bool planAndExecute(moveit::planning_interface::MoveGroupInterface& move_group_interface, moveit::planning_interface::MoveGroupInterface::Plan& plan) {
        bool success = static_cast<bool>(move_group_interface.plan(plan));
        if (success) {
            auto execute_result = move_group_interface.execute(plan);
            return (execute_result == moveit::core::MoveItErrorCode::SUCCESS);
        } 
        return false;
    }
#else
// 이전
    bool planAndExecute(moveit::planning_interface::MoveGroupInterface& move_group_interface, moveit::planning_interface::MoveGroupInterface::Plan& plan) {
       bool success = static_cast<bool>(move_group_interface.plan(plan));
       if (success) {
           move_group_interface.execute(plan);
           return true;
       } else {
           return false;
       }
    }
#endif

#if NEW
// 수정    
    geometry_msgs::msg::Quaternion multiply(const geometry_msgs::msg::Quaternion& q1, const geometry_msgs::msg::Quaternion& q2) {
        tf2::Quaternion tf_q1(q1.x, q1.y, q1.z, q1.w);
        tf2::Quaternion tf_q2(q2.x, q2.y, q2.z, q2.w);
        tf2::Quaternion tf_result = tf_q1 * tf_q2;
    
        geometry_msgs::msg::Quaternion result;
        result.x = tf_result.x();
        result.y = tf_result.y();
        result.z = tf_result.z();
        result.w = tf_result.w();
        return result;
    }
#else
// 이전
    geometry_msgs::msg::Quaternion multiply(const geometry_msgs::msg::Quaternion& q2, const geometry_msgs::msg::Quaternion& q1) {
       geometry_msgs::msg::Quaternion out;
       out.w = q2.w*q1.w - q2.x*q1.x - q2.y*q1.y - q2.z*q1.z;
       out.x = q2.w*q1.x + q2.x*q1.w + q2.y*q1.z - q2.z*q1.y;
       out.y = q2.w*q1.y - q2.x*q1.z + q2.y*q1.w + q2.z*q1.x;
       out.z = q2.w*q1.z + q2.x*q1.y - q2.y*q1.x + q2.z*q1.w;
       return out;
    }
#endif


private:
    void handleMoveitControl(const std::shared_ptr<turtlebot_cosmo_interface::srv::MoveitControl::Request> req,
                             std::shared_ptr<turtlebot_cosmo_interface::srv::MoveitControl::Response> res) {
        
        // moveit core 의 arm 그룹을 사용하기위한 선언
        using moveit::planning_interface::MoveGroupInterface;
        auto arm_interface = MoveGroupInterface(shared_from_this(), "arm");
        arm_interface.setPlanningPipelineId("move_group");
        arm_interface.setPlanningTime(10.0); 
        // arm_interface.setGoalPositionTolerance(0.01);
        // arm_interface.setGoalOrientationTolerance(0.05);

        arm_interface.setGoalPositionTolerance(0.005);
        arm_interface.setGoalOrientationTolerance(0.025);
        
        // arm_interface.setGoalPositionTolerance(0.0025);
        // arm_interface.setGoalOrientationTolerance(0.0125);

        // moveit core 의 gripper 그룹을 사용하기위한 선언
        auto gripper_interface = MoveGroupInterface(shared_from_this(), "gripper");
        

        // 좌표를 통한 이동 
#if NEW
// 수정
        if (req->cmd == 0) {
            std::vector<geometry_msgs::msg::Pose> waypoints(req->waypoints.poses.begin(), req->waypoints.poses.end());
            if (waypoints.empty()) {
                RCLCPP_WARN(this->get_logger(), "No waypoints provided for cmd 0.");
                res->response = false;
                return;
            }
    
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            auto _pose = waypoints[0];
            auto current_pose = arm_interface.getCurrentPose().pose;

            float target_z = _pose.position.z;
            float target_x = _pose.position.x;
            float target_y = _pose.position.y;
            //double yaw = atan2(target_y, target_x);
            double yaw = atan2(target_y - current_pose.position.y, target_x - current_pose.position.x);

            geometry_msgs::msg::Quaternion multiply_quaternion = multiply(rpyToQuaternion(0, 0, yaw), rpyToQuaternion(0, PI/2, 0));

            geometry_msgs::msg::Pose target_pose;
            target_pose.orientation = multiply_quaternion;
            target_pose.position.x = target_x;
            target_pose.position.y = target_y;
            target_pose.position.z = target_z;

            arm_interface.setPoseTarget(target_pose);
            res->response = planAndExecute(arm_interface, plan);
        }

#else
// 이전
        if (req->cmd == 0) {
           std::vector<geometry_msgs::msg::Pose> waypoints;
           for (const auto &pose : req->waypoints.poses) {
               waypoints.push_back(pose);
           }
           auto _pose = waypoints[0];
           moveit::planning_interface::MoveGroupInterface::Plan plan;
           auto current_pose = arm_interface.getCurrentPose().pose;
           float target_z = _pose.position.z;
           float target_x = _pose.position.x;
           float target_y = _pose.position.y;
           double yaw = atan2(target_y, target_x);
           std::cout<<"yaw : "<<yaw <<std::endl;  
           // 원점 좌표에서 로봇팔 좌표의 축차이를 직접계산해서 넣어줌
           // geometry_msgs::msg::Quaternion multiply_quaternion = multiply(rpyToQuaternion(0, 0, yaw),current_pose.orientation);

           geometry_msgs::msg::Quaternion multiply_quaternion = multiply(rpyToQuaternion(0, 0, yaw),rpyToQuaternion(0, PI/2, 0));
           std::cout << "multiply_quaternion: " << multiply_quaternion.x << " " << multiply_quaternion.y << " " << multiply_quaternion.z << " " << multiply_quaternion.w << std::endl;

           current_pose = arm_interface.getCurrentPose().pose;
           auto target_pose = [multiply_quaternion, target_x, target_y, target_z]{
               geometry_msgs::msg::Pose msg;
               msg.orientation = multiply_quaternion;
               msg.position.x = target_x;
               msg.position.y = target_y;
               msg.position.z = target_z; 
               return msg;
           }();
           arm_interface.setPoseTarget(target_pose);
           planAndExecute(arm_interface, plan);
        }
#endif

        else if (req->cmd == 1) {
            arm_interface.setNamedTarget(req->posename);
            // res->response = (arm_interface.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            res->response = (arm_interface.move() == moveit::core::MoveItErrorCode::SUCCESS);

        } 

        else if (req->cmd == 2) {
            gripper_interface.setNamedTarget(req->posename);
            // res->response = (gripper_interface.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            res->response = (gripper_interface.move() == moveit::core::MoveItErrorCode::SUCCESS);

        } 

        else if (req->cmd == 3){
            std::vector<geometry_msgs::msg::Pose> waypoints;
            for (const auto &pose : req->waypoints.poses) {
                waypoints.push_back(pose);
            }
            auto _pose = waypoints[0];
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            auto current_pose = arm_interface.getCurrentPose().pose;
            float target_z = _pose.position.z;
            float target_x = _pose.position.x;
            float target_y = _pose.position.y;
            double yaw = atan2(target_x, target_y);  
            std::cout<<"yaw"<< yaw << std::endl;
            geometry_msgs::msg::Quaternion multiply_quaternion = multiply(rpyToQuaternion(0, 0, yaw),rpyToQuaternion(0, 0, PI/2));
            std::cout << "multiply_quaternion: " << multiply_quaternion.x << " " << multiply_quaternion.y << " " << multiply_quaternion.z << " " << multiply_quaternion.w << std::endl;

            current_pose = arm_interface.getCurrentPose().pose;
            auto target_pose = [multiply_quaternion, target_x, target_y, target_z]{
                geometry_msgs::msg::Pose msg;
                msg.orientation = multiply_quaternion;
                msg.position.x = target_x;
                msg.position.y = target_y;
                msg.position.z = target_z; 
                return msg;
            }();
            arm_interface.setPoseTarget(target_pose);
            planAndExecute(arm_interface, plan);
        }


        else if (req->cmd == 4){
            std::vector<geometry_msgs::msg::Pose> waypoints;
            for (const auto &pose : req->waypoints.poses) {
                waypoints.push_back(pose);
            }
            auto _pose = waypoints[0];
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            auto current_pose = arm_interface.getCurrentPose().pose;
            float target_z = _pose.position.z;
            float target_x = _pose.position.x;
            float target_y = _pose.position.y;
            float quaternion_x = _pose.orientation.x;
            float quaternion_y = _pose.orientation.y;
            float quaternion_z = _pose.orientation.z;
            float quaternion_w = _pose.orientation.w;
            
            // double yaw = atan2(target_x, target_y);  
            // std::cout<<yaw;

            current_pose = arm_interface.getCurrentPose().pose;
            auto target_pose = [quaternion_x, quaternion_y, quaternion_z, quaternion_w, target_x, target_y, target_z]{
                geometry_msgs::msg::Pose msg;
                msg.orientation.x = quaternion_x;
                msg.orientation.y = quaternion_y;
                msg.orientation.z = quaternion_z;
                msg.orientation.w = quaternion_w;
                msg.position.x = target_x;
                msg.position.y = target_y;
                msg.position.z = target_z; 
                return msg;
            }();
            arm_interface.setPoseTarget(target_pose);
            planAndExecute(arm_interface, plan);
        }

        else if (req->cmd==9){
            printCurrentPose();
        }

#if NEW
//수정
        else {
            RCLCPP_WARN(this->get_logger(), "Received unknown command: %d", req->cmd);
            res->response = false;
        }
#else
//이전
        else {
           res->response = false;
        }
#endif
    }

    rclcpp::Service<turtlebot_cosmo_interface::srv::MoveitControl>::SharedPtr service_;
    rclcpp::Node::SharedPtr node_;                                         // Additional ROS node pointer
    moveit::planning_interface::MoveGroupInterface move_group_interface_;  // MoveIt interface for controlling the arm
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;  // Single-threaded executor
    std::thread executor_thread_;                                          // Thread to run the executor
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);    // Allow automatic parameter declaration
    node_options.use_intra_process_comms(false);                           // Disable intra-process communication
    auto node = std::make_shared<TurtlebotArmController>(node_options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}