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
    sub_theta1 = nh_.subscribe("/position_1", 10, &DynamixelControl::Callback_theta1, this);
    sub_theta2 = nh_.subscribe("/position_2", 10, &DynamixelControl::Callback_theta2, this);

    sub_left   = nh_.subscribe("/motor_left", 10, &DynamixelControl::Callback_left, this);
    sub_center = nh_.subscribe("/motor_center", 10, &DynamixelControl::Callback_center, this);
    sub_right  = nh_.subscribe("/motor_right", 10, &DynamixelControl::Callback_right, this);

    pub_dir    = nh_.advertise<std_msgs::Int32>("/radar_dir", 10);
    
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
  
  
  /* Jammer-On interrupt & Moving Jammer Antenna */
  /* Need to setting */
  
  
  /* Jammer Motor pos Callback Fnc */
  void Callback_theta1(const std_msgs::Int32::ConstPtr& msg)
  {
    ROS_INFO("theta3: %d",msg->data);
    setPosition(3,msg->data); 
  }
  
  void Callback_theta2(const std_msgs::Int32::ConstPtr& msg)
  {
    ROS_INFO("theta4: %d",msg->data);
    setPosition(4,msg->data);
  }
  
  void Callback_left(const std_msgs::Int32::ConstPtr& msg)
  {
    ros::Duration(5).sleep();

    int R = 0;
    std_msgs::Int32 r_msg;
    r_msg.data = R;
    pub_dir.publish(r_msg);

    // signal processing delay
    ros::Duration(1).sleep();

    ser.write("H");

    int theta3 = 1650; // 150
    int theta4 = 1050; // 100

	//ROS_INFO("theta3: %d, theta4: %d", theta3, theta4);
	setPosition(3,theta3); 
	setPosition(4,theta4);
  }

  void Callback_center(const std_msgs::Int32::ConstPtr& msg)
  {
    ros::Duration(5).sleep();

    int R = 1;
    std_msgs::Int32 r_msg;
    r_msg.data = R;
    pub_dir.publish(r_msg);

    // signal processing delay
    ros::Duration(1).sleep();
    
    ser.write("H");

    int theta3 = 1300; // 115
    int theta4 = 1050; // 100

	//ROS_INFO("theta3: %d, theta4: %d", theta3, theta4);
	setPosition(3,theta3); 
	setPosition(4,theta4);
  }

  void Callback_right(const std_msgs::Int32::ConstPtr& msg)
  {
    ros::Duration(5).sleep();

    int R = 2;
    std_msgs::Int32 r_msg;
    r_msg.data = R;
    pub_dir.publish(r_msg);

    // signal processing delay
    ros::Duration(1).sleep();
    
    ser.write("H");

    int theta3 = 950;  // 80
    int theta4 = 1050; // 100
    //int theta3 = 1150;
    //int theta4 = 950;

	//ROS_INFO("theta3: %d, theta4: %d", theta3, theta4);
	setPosition(3,theta3); 
	setPosition(4,theta4);
  }
  

  /* Set Position & Set Speed Fnc */
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
  ros::Subscriber sub_left;
  ros::Subscriber sub_center;
  ros::Subscriber sub_right;

  ros::Publisher pub_dir;
  
  serial::Serial ser;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "jammer_unit_node");
  DynamixelControl jammer_unit;
  ros::spin();
  return 0;
}
