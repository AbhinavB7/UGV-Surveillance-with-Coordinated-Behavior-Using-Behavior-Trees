#ifndef CHECK_FOR_PERSON_HPP_
#define CHECK_FOR_PERSON_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "behaviortree_cpp_v3/condition_node.h"
#include <string>

class CheckForPerson : public BT::ConditionNode
{
public:
    // Constructor
    CheckForPerson(const std::string &name, const BT::NodeConfiguration &config, rclcpp::Node::SharedPtr node_ptr);

    // Required to define the node configuration
    static BT::PortsList providedPorts()
    {
        return {};
    }

    // The tick function that gets called periodically
    BT::NodeStatus tick() override;

private:
    // Node shared pointer for ROS2 communication
    rclcpp::Node::SharedPtr node_ptr_;

    // ROS2 subscription to the /person topic
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;

    // Timer to periodically check the detection status
    rclcpp::TimerBase::SharedPtr timer_;

    // Flag to indicate if a person was detected
    bool person_detected_;

    // Last recorded detection status
    BT::NodeStatus detection_status_;

    // The last pose message received when a person was detected
    geometry_msgs::msg::PoseStamped last_pose_;

    // Method to write the pose data to a YAML file
    void writePoseToFile(const std::string &filename, const geometry_msgs::msg::PoseStamped &pose);
};

#endif // CHECK_FOR_PERSON_HPP_
