#include "ros/ros.h"
#include "dynamixel_workbench_msgs/DynamixelCommand.h"
#include "dynamixel_workbench_msgs/DynamixelStateList.h"

class DynamixelControl
{
public:
  DynamixelControl()
  {
    client = nh_.serviceClient<dynamixel_workbench_msgs::DynamixelCommand>("/dynamixel_workbench/dynamixel_command");
    position_sub_ = nh_.subscribe("/dynamixel_workbench/dynamixel_state", 10, &DynamixelControl::positionCallback, this);

    ros::Duration(0.5).sleep();
    setPosition(2, 0);
  }

  void positionCallback(const dynamixel_workbench_msgs::DynamixelStateList::ConstPtr& msg)
  {
    for (const auto& state : msg->dynamixel_state)
    {
      if (state.id == 2)
      {
        int current_position = state.present_position;
        ROS_INFO("Current Position of ID %d: %d", state.id, current_position);
        if (current_position <= 0)
        {
          setPosition(2, 2048);
        }
        else if (current_position >= 2048)
        {
          setPosition(2,0);
        }
      }
    }
  }

  void setPosition(int id, int position)
  {
    dynamixel_workbench_msgs::DynamixelCommand srv;
    srv.request.command = "";
    srv.request.id = id;
    srv.request.addr_name = "Goal_Position";
    srv.request.value = position;

    client.call(srv);
  }

private:
  ros::NodeHandle nh_;
  ros::ServiceClient client;
  ros::Subscriber position_sub_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "jammer_unit_node");
  DynamixelControl jammer_unit;
  ros::spin();
  return 0;
}
