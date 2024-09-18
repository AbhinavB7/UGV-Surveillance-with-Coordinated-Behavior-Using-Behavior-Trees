#include "rclcpp/rclcpp.hpp"
#include "person_node.h"
#include <chrono>
#include <fstream>
#include <yaml-cpp/yaml.h>

CheckForPerson::CheckForPerson(const std::string &name, const BT::NodeConfiguration &config, rclcpp::Node::SharedPtr node_ptr)
    : BT::ConditionNode(name, config), node_ptr_(node_ptr), person_detected_(false), detection_status_(BT::NodeStatus::RUNNING)
{
    subscription_ = node_ptr_->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/person", 10, [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            last_pose_ = *msg;
            person_detected_ = true;
        });

    // Set up a timer to check the detection state periodically
    timer_ = node_ptr_->create_wall_timer(
        std::chrono::milliseconds(100),  
        [this]() {
            if (person_detected_) {
                // RCLCPP_INFO(node_ptr_->get_logger(), "Person detected.");
                
                detection_status_ = BT::NodeStatus::SUCCESS;
                
            } else {
                // RCLCPP_INFO(node_ptr_->get_logger(), "No person detected.");
                detection_status_ = BT::NodeStatus::RUNNING;
            }
        });
}

BT::NodeStatus CheckForPerson::tick()
{
    if (detection_status_ == BT::NodeStatus::SUCCESS)
    {
        if (last_pose_.header.frame_id.empty())  // Example validation; adjust as needed
        {
            RCLCPP_WARN(node_ptr_->get_logger(), "Invalid person pose.");
            return BT::NodeStatus::FAILURE;
        }
        
        // If a person was detected, set the pose in the blackboard
        setOutput<geometry_msgs::msg::PoseStamped>("person_pose", last_pose_);
        RCLCPP_INFO(node_ptr_->get_logger(), "Person pose set in the blackboard.");
        RCLCPP_INFO(node_ptr_->get_logger(), "The person pose is %f, %f, %f",
                    last_pose_.pose.position.x, last_pose_.pose.position.y, last_pose_.pose.position.z);

        // Write pose to a YAML file
        writePoseToFile(node_ptr_->get_parameter("person_file").as_string(), last_pose_);

        // Return SUCCESS to stop further execution
        return BT::NodeStatus::SUCCESS;
    }
    
    // If no person was detected, continue running
    return BT::NodeStatus::RUNNING;
}

void CheckForPerson::writePoseToFile(const std::string &filename, const geometry_msgs::msg::PoseStamped &pose)
{
    // Create a YAML node and populate it with the pose data
    YAML::Node node;
    

    YAML::Node pose_node;
    pose_node.push_back(pose.pose.position.x);
    pose_node.push_back(pose.pose.position.y);
    pose_node.push_back(pose.pose.position.z);
    
    node["person_pose"] = pose_node;

    // Write the YAML node to the specified file
    std::ofstream fout(filename);
    fout << node;
    fout.close();
    
    RCLCPP_INFO(node_ptr_->get_logger(), "Pose written to file: %s", filename.c_str());
}
