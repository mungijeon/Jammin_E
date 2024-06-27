#include "ros/ros.h"
#include "dynamixel_workbench_msgs/DynamixelStateList.h"

void dynamixelStateCallback(const dynamixel_workbench_msgs::DynamixelStateList::ConstPtr& msg)
{
  for (const auto& state : msg->dynamixel_state)
  {
    ROS_INFO("ID: %d, Position: %d", state.id, state.present_position);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "postition_callback_node");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("/dynamixel_workbench/dynamixel_state", 10, dynamixelStateCallback);

  ros::spin();

  return 0;
}
