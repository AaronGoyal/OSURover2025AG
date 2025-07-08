#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <Eigen/SVD>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/joint_model_group.h>


using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class Manipulability : public rclcpp::Node//, public std::enable_shared_from_this<Manipulability>
{
    public:
        Manipulability()
        : Node("manipulability")
        {
            // Parameters
            this->declare_parameter("planning_group", "rover_arm");
            this->declare_parameter("end_effector_link", "arm_gripper");
            this->declare_parameter("manipulability_threshold", 1e-3);

            this->get_parameter("planning_group", planning_group_);
            this->get_parameter("end_effector_link", end_effector_link_);
            this->get_parameter("manipulability_threshold", manipulability_threshold_);
            
            

        }
        void init(){
            // Load robot model
            robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(
                this->shared_from_this(),
                "/move_group/robot_description",
                "/move_group/robot_description_semantic"
            );
            robot_model_ = robot_model_loader_->getModel();
            if (!robot_model_) {
            RCLCPP_FATAL(this->get_logger(), "Failed to load robot model");
            rclcpp::shutdown();
            return;
            }

            joint_model_group_ = robot_model_->getJointModelGroup(planning_group_);
            if (!joint_model_group_) {
            RCLCPP_FATAL(this->get_logger(), "Planning group '%s' not found", planning_group_.c_str());
            rclcpp::shutdown();
            return;
            }

            robot_state_ = std::make_shared<moveit::core::RobotState>(robot_model_);
            robot_state_->setToDefaultValues();

            joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states", 10,
                std::bind(&Manipulability::joint_states_callback, this, std::placeholders::_1)
            );
            
            manip_index_pub_ = this->create_publisher<std_msgs::msg::String>(
                "manip_index", 10
            );
        }

    private:
        
        std::string planning_group_;
        std::string end_effector_link_;
        double manipulability_threshold_;

        //Moveit params
        robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
        moveit::core::RobotModelPtr robot_model_;
        moveit::core::RobotStatePtr robot_state_;
        const moveit::core::JointModelGroup* joint_model_group_;

        //Pubs
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr manip_index_pub_;

        //Subs
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    
        void joint_states_callback(sensor_msgs::msg::JointState::SharedPtr msg) const{
            robot_state_->setVariablePositions(msg->name, msg->position);
            robot_state_->update();

            Eigen::MatrixXd jacobian;
            robot_state_->getJacobian(
            joint_model_group_,
            robot_state_->getLinkModel(end_effector_link_),
            Eigen::Vector3d::Zero(),  // Ref point at origin
            jacobian);

            Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian);
            auto singular_values = svd.singularValues();
            double manipulability = singular_values.prod();
            auto message = std_msgs::msg::String();
            message.data = std::to_string(manipulability);
            manip_index_pub_->publish(message);


        }        
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto manip_node = std::make_shared<Manipulability>();
  manip_node->init();

  rclcpp::spin(manip_node);
  rclcpp::shutdown();
  return 0;
}