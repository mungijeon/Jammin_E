#include "ros/ros.h"
#include "dynamixel_workbench_msgs/DynamixelStateList.h"
#include "dynamixel_workbench_msgs/DynamixelCommand.h"
#include <csignal>

class DynamixelControl
{
public:
  int motor_id = 2;
  int motor_velocity = 10;
  int direction = 1;

  DynamixelControl()
  {
    client = nh.serviceClient<dynamixel_workbench_msgs::DynamixelCommand>("/dynamixel_workbench/dynamixel_command");
    sub = nh.subscribe("/dynamixel_workbench/dynamixel_state", 10, &DynamixelControl::Callback, this);
    ros::Duration(1.0).sleep();
    setVelocity(motor_id, direction * motor_velocity);
  }

  void setVelocity(int id, int velocity)
  {
    dynamixel_workbench_msgs::DynamixelCommand srv;
    srv.request.command = "";
    srv.request.id = id;
    srv.request.addr_name = "Goal_Velocity";
    srv.request.value = velocity;

    client.call(srv);
  }

  void Callback(const dynamixel_workbench_msgs::DynamixelStateList::ConstPtr& msg)
  {
    for (const auto& state : msg->dynamixel_state)
    {
      if (state.id == motor_id)
      {
        ROS_INFO("ID: %d, Position: %d", state.id, state.present_position);
        if (state.present_position == 0 || state.present_position == 1800)
        {
          //ros::Duration(1.0).sleep();
          direction *= -1;
          setVelocity(motor_id, direction * motor_velocity);
          ROS_INFO("Direction changed: %d", direction);
        }
      }
    }
  }

  void stopMotor()
  {
    setVelocity(motor_id, 0);
  }

private:
  ros::NodeHandle nh;
  ros::ServiceClient client;
  ros::Subscriber sub;
};

// 傈开 按眉肺 包府
DynamixelControl* radar_unit_ptr;

void shutdownCallback(int sig)
{
  radar_unit_ptr->stopMotor();
  ros::shutdown();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "radar_unit_node", ros::init_options::NoSigintHandler);

  DynamixelControl radar_unit;
  radar_unit_ptr = &radar_unit;

  signal(SIGINT, shutdownCallback);

  ros::spin();

  return 0;
}