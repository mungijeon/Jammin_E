#include "ros/ros.h"
#include "dynamixel_workbench_msgs/DynamixelCommand.h"
#include "dynamixel_workbench_msgs/DynamixelStateList.h"
#include "std_msgs/Int32.h"

class DynamixelControl
{
public:
  DynamixelControl()
  {
    client = nh_.serviceClient<dynamixel_workbench_msgs::DynamixelCommand>("/dynamixel_workbench/dynamixel_command");
    sub = nh_.subscribe("/dynamixel_workbench/dynamixel_state", 10, &DynamixelControl::Callback, this);
    sub_r = nh_.subscribe("/rf_layer", 10, &DynamixelControl::RCallback, this);    
    pub_theta1 = nh_.advertise<std_msgs::Int32>("/topic_theta1", 10);
    pub_theta2 = nh_.advertise<std_msgs::Int32>("/topic_theta2", 10);
    pub_r = nh_.advertise<std_msgs::Int32>("/topic_r", 10);

    /*********************************************************************
      TOPIC LIST

      sub r      : /rf_layer        from matlab

      pub theta1 : /topic_theta1    to simulink
      pub theta2 : /topic_theta2    to simulink
      pub r      : /topic_r         to simulink
    *********************************************************************/

    ros::Duration(0.5).sleep();
    setSpeed(1,15);
    setSpeed(2,40);
    setPosition(1, 2048);
    setPosition(2, 383);

    /*********************************************************************
      INIT VALUE
      
      SPEED

      Motor1(xy) : 40
      Motor2(z)  : 300

      POSITION

      Motor1(xy) : 1000
      Motor2(z)  : 0
    *********************************************************************/
  }

  void Callback(const dynamixel_workbench_msgs::DynamixelStateList::ConstPtr& msg)
  {
    for (const auto& state : msg->dynamixel_state)
    {
      if (state.id == 1)
      {
        current_position1 = state.present_position;
        //ROS_INFO("Current Position of ID %d: %d", state.id, current_position1);
        if (current_position1 <= 700)
        {
          //setPosition(1, 1700);
          //setPosition(2,400);
        }
        else if (current_position1 >= 1600)
        {
          //setPosition(1,600);
          //setPosition(2,700);
        }
      }
      else if (state.id == 2)
      {
        current_position2 = state.present_position;
      }
    }
  }

  void RCallback(const std_msgs::Int32::ConstPtr& msg)
  {
    int R = msg->data;
    ROS_INFO("R: %d", R);
    ROS_INFO("theta1: %d", current_position1);
    ROS_INFO("theta2: %d", current_position2);

    std_msgs::Int32 r_msg;
    r_msg.data = R;
    pub_r.publish(r_msg);

    std_msgs::Int32 theta1_msg;
    theta1_msg.data = current_position1;
    pub_theta1.publish(theta1_msg);

    std_msgs::Int32 theta2_msg;
    theta2_msg.data = current_position2;
    pub_theta2.publish(theta2_msg);
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
  ros::Subscriber sub;
  ros::Subscriber sub_r;
  ros::Publisher pub_theta1;
  ros::Publisher pub_theta2;
  ros::Publisher pub_r;

  int current_position1 = 0;
  int current_position2 = 0;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "radar_unit_node");
  DynamixelControl radar_unit;
  ros::spin();
  return 0;
}
