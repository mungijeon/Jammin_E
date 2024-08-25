#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "dynamixel_workbench_msgs/DynamixelCommand.h"

class DynamixelControl
{
public:
  DynamixelControl()
  {
    client = nh_.serviceClient<dynamixel_workbench_msgs::DynamixelCommand>("/dynamixel_workbench/dynamixel_command");

    sub_theta1 = nh_.subscribe("/position_1", 10, &DynamixelControl::commandCallback1, this);
    sub_theta2 = nh_.subscribe("/position_2", 10, &DynamixelControl::commandCallback2, this);
    
    /*********************************************************************
      TOPIC LIST

      sub theta1 : /position_1    from simulink
      sub theta2 : /position_2    from simulink
    *********************************************************************/

  }

  void commandCallback1(const std_msgs::Int32::ConstPtr& msg)
  {
    ROS_INFO("Received tr_theta1: %d", msg->data);
    setPosition(3, msg->data);
  }
  void commandCallback2(const std_msgs::Int32::ConstPtr& msg)
  {
    ROS_INFO("Received tr_theta2: %d", msg->data);
    setPosition(4, msg->data);
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
  ros::Subscriber sub_theta1;
  ros::Subscriber sub_theta2;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "jammer_unit_node");
  DynamixelControl jammer_unit;
  ros::spin();
  return 0;
}
