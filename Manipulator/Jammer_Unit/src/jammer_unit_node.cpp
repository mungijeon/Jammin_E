#include "ros/ros.h"
#include "std_msgs/Int32MultiArray.h"
#include "dynamixel_workbench_msgs/DynamixelCommand.h"

class DynamixelControl
{
public:
  DynamixelControl()
  {
    command_client_ = nh_.serviceClient<dynamixel_workbench_msgs::DynamixelCommand>("/dynamixel_workbench/dynamixel_command");
    sub_ = nh_.subscribe("dynamixel_positions", 10, &DynamixelControl::commandCallback, this);
  }

  void commandCallback(const std_msgs::Int32MultiArray::ConstPtr& msg)
  {
    setPosition(1, msg->data[0]);
    setPosition(2, msg->data[1]);
  }

  void setPosition(int id, int position)
  {
    dynamixel_workbench_msgs::DynamixelCommand srv;
    srv.request.command = "";
    srv.request.id = id;
    srv.request.addr_name = "Goal_Position";
    srv.request.value = position;

    command_client_.call(srv);
  }

private:
  ros::NodeHandle nh_;
  ros::ServiceClient command_client_;
  ros::Subscriber sub_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "jammer_unit_node");
  DynamixelControl jammer_unit;
  ros::spin();
  return 0;
}
