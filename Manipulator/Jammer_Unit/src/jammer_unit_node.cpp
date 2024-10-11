#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "dynamixel_workbench_msgs/DynamixelCommand.h"

#include "serial/serial.h"

class DynamixelControl
{
public:
  DynamixelControl()
  {
    /*********************************************************************
      TOPIC LIST

      sub theta1 : /position_1    from simulink
      sub theta2 : /position_2    from simulink
    *********************************************************************/
    client     = nh_.serviceClient<dynamixel_workbench_msgs::DynamixelCommand>("/dynamixel_workbench/dynamixel_command");
    
    sub_theta1 = nh_.subscribe("/position_1", 10, &DynamixelControl::Callback1, this);
    sub_theta2 = nh_.subscribe("/position_2", 10, &DynamixelControl::Callback2, this);
    
    /*********************************************************************
      INIT VALUE
      
      SPEED

      Motor3(xy) : 150
      Motor4(z)  : 150

      POSITION

      Motor3(xy) : 1300
      Motor4(z)  : 1600
    *********************************************************************/
    setSpeed(3,150);
    setSpeed(4,150);
    setPosition(3,1300); // 115
    setPosition(4,1600); // 140
    
    /* Relay Module: Set Low voltage */
    try
    {
      ser.setPort("/dev/ttyACM0");
      ser.setBaudrate(9600);
      serial::Timeout to = serial::Timeout::simpleTimeout(1000);
      ser.setTimeout(to);
      ser.open();
      
      if (ser.isOpen())
      {
        ROS_INFO("Serial Port opened successfully.");
      }
    }
      catch (serial::IOException& e)
      {
        ROS_ERROR_STREAM("Unable to open port.");
      }
      
    ser.write("L");
  }


  /* Callback Function */
  void Callback1(const std_msgs::Int32::ConstPtr& msg)
  {
    ser.write("H");
    setPosition(3, msg->data);
  }
  void Callback2(const std_msgs::Int32::ConstPtr& msg)
  {
    //ser.write("H");
    setPosition(4, msg->data);
  }
  
  
  /* Set Position & Speed */
  void setPosition(int id, int position)
  {
    dynamixel_workbench_msgs::DynamixelCommand srv;
    srv.request.command = "";
    srv.request.id = id;
    srv.request.addr_name = "Goal_Position";
    srv.request.value = position;

    client.call(srv);
  }
  
  void setSpeed(int id, int speed)
  {
    dynamixel_workbench_msgs::DynamixelCommand srv;
    srv.request.command = "";
    srv.request.id = id;
    srv.request.addr_name = "Profile_Velocity";
    srv.request.value = speed;

    client.call(srv);
  }

private:
  ros::NodeHandle nh_;
  ros::ServiceClient client;
  
  ros::Subscriber sub_theta1;
  ros::Subscriber sub_theta2;
  
  serial::Serial ser;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "jammer_unit_node");
  DynamixelControl jammer_unit;
  ros::spin();
  return 0;
}
