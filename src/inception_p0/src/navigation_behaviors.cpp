#include "navigation_behaviors.h"
#include "yaml-cpp/yaml.h"
#include <string>

GoToPose::GoToPose(const std::string &name, const BT::NodeConfiguration &config, rclcpp::Node::SharedPtr node_ptr)
    : BT::StatefulActionNode(name, config), node_ptr_(node_ptr)
{
  ns_ = node_ptr_->get_parameter("robot_name").as_string();

  action_client_ptr_ = rclcpp_action::create_client<NavigateToPose>(node_ptr_, "/"+ ns_+"/navigate_to_pose");
  done_flag_ = false;
}

BT::PortsList GoToPose::providedPorts()
{
  return {
      BT::InputPort<std::string>("loc"),
  };
}

BT::NodeStatus GoToPose::onStart()
{
  std::string location;

  // If person_pose is not available, fall back to the YAML-based location
  if (!getInput<std::string>("loc", location))
  {
    throw BT::RuntimeError("Missing required input [loc]");
  }

  RCLCPP_INFO(node_ptr_->get_logger(), "Not Using location from blackboard.");
  
  
  YAML::Node locations; 

  std::string stripped_location = location.substr(8);  // Strips "location" prefix
  
  if (stripped_location == "7")
  {
    RCLCPP_INFO(node_ptr_->get_logger(), "Loc = 7");
    locations = YAML::LoadFile(node_ptr_->get_parameter("person_file").as_string());

    if (!locations["person_pose"])
    {
      throw BT::RuntimeError("person_pose not found in person_file");
    }

    std::vector<float> pose = locations["person_pose"].as<std::vector<float>>();

    RCLCPP_INFO(node_ptr_->get_logger(), "loc=7, person_pose: [%.2f, %.2f, %.2f]", pose[0], pose[1], pose[2]);

    // Construct the goal message using the pose from person_pose
    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.pose.position.x = pose[0];
    goal_msg.pose.pose.position.y = pose[1];
    tf2::Quaternion q;
    q.setRPY(0, 0, pose[2]);
    goal_msg.pose.pose.orientation = tf2::toMsg(q);

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback = std::bind(&GoToPose::nav_to_pose_callback, this, std::placeholders::_1);
    action_client_ptr_->async_send_goal(goal_msg, send_goal_options);

    RCLCPP_INFO(node_ptr_->get_logger(), "Sent Goal to Nav2 for person_pose");
    
    return BT::NodeStatus::SUCCESS;

  }

  else

  {
    RCLCPP_INFO(node_ptr_->get_logger(), "Using location file for location_tb1_%s", stripped_location.c_str());
    locations = YAML::LoadFile(node_ptr_->get_parameter("location_file").as_string());

    std::string tb1_location = "location_" + ns_ + "_" + stripped_location;
    
    if (!locations[tb1_location])
    {
      throw BT::RuntimeError("Location not found in : ", tb1_location);
    }

    std::vector<float> pose = locations[tb1_location].as<std::vector<float>>();
    RCLCPP_INFO(node_ptr_->get_logger(), "Location %s: [%.2f, %.2f, %.2f]", stripped_location.c_str(), pose[0], pose[1], pose[2]);

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.pose.position.x = pose[0];
    goal_msg.pose.pose.position.y = pose[1];
    tf2::Quaternion q;
    q.setRPY(0, 0, pose[2]);
    goal_msg.pose.pose.orientation = tf2::toMsg(q);

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback = std::bind(&GoToPose::nav_to_pose_callback, this, std::placeholders::_1);
    action_client_ptr_->async_send_goal(goal_msg, send_goal_options);

    RCLCPP_INFO(node_ptr_->get_logger(), "Sent Goal to Nav2 for location %s", stripped_location.c_str());
  }

  return BT::NodeStatus::RUNNING;
  }


BT::NodeStatus GoToPose::onRunning()
{
  return done_flag_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
}

void GoToPose::nav_to_pose_callback(const GoalHandleNav::WrappedResult &result)
{
  if (result.result)
  {
    done_flag_ = true;
  }
}

void GoToPose::onHalted()
{
  RCLCPP_INFO(node_ptr_->get_logger(), "Halting navigation.");
}
