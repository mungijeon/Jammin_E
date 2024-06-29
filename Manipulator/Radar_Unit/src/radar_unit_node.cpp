#include "ros/ros.h"
#include "dynamixel_workbench_msgs/DynamixelCommand.h"
// dynamixel_workbench_msgs::DynamixelStateList �޼��� Ÿ���� ����ϱ� ����
// �̴� ���̳��ͼ� ���� ������ ����
#include "dynamixel_workbench_msgs/DynamixelStateList.h"
#include "std_msgs/Int32.h"       // ���� R ���� ���� �޽��� Ÿ��

class DynamixelControl
{
public:
  DynamixelControl()
  {
    // ���� Ŭ���̾�Ʈ ����: dynamixel_command ���񽺸� ȣ���ϱ� ���� Ŭ���̾�Ʈ�� ����
    client = nh_.serviceClient<dynamixel_workbench_msgs::DynamixelCommand>("/dynamixel_workbench/dynamixel_command");
    // ������ ����: dynamixel_state ������ �����ϰ�, �ݹ� �Լ��� ����
    sub = nh_.subscribe("/dynamixel_workbench/dynamixel_state", 10, &DynamixelControl::Callback, this);
    // topic1�� ���� ����
    sub_r = nh_.subscribe("rf_layer", 10, &DynamixelControl::RCallback, this);    
    // topic2�� ���� ����
    pub_theta1 = nh_.advertise<std_msgs::Int32>("topic_theta1", 10);
    pub_theta2 = nh_.advertise<std_msgs::Int32>("topic_theta2", 10);
    pub_r = nh_.advertise<std_msgs::Int32>("topic_r", 10);

    ros::Duration(0.5).sleep();
    setSpeed(1,40);
    setSpeed(2,300);
    setPosition(1, 1000);
    setPosition(2, 0);
  }

  // �ݹ� �Լ� ����: dynamixel_state �޼����� ó��
  void Callback(const dynamixel_workbench_msgs::DynamixelStateList::ConstPtr& msg)
  {
    // dynamixel_state ����Ʈ ���� �� ���� �޼����� �ݺ� ó��
    for (const auto& state : msg->dynamixel_state)
    {
      if (state.id == 1)
      {
        current_position1 = state.present_position;
        ROS_INFO("Current Position of ID %d: %d", state.id, current_position1);
        if (current_position1 <= 1024)
        {
          setPosition(1, 3100);
          setPosition(2,1024);
        }
        else if (current_position1 >= 3072)
        {
          setPosition(1,1000);
          setPosition(2,0);
        }
      }
      else if (state.id == 2)
      {
        current_position2 = state.present_position;
      }
    }
  }

  // R ���� �޴� �ݹ� �Լ�
  void RCallback(const std_msgs::Int32::ConstPtr& msg)
  {
    int R = msg->data;
    ROS_INFO("R: %d", R);
    ROS_INFO("theta1: %d", current_position1);
    ROS_INFO("theta2: %d", current_position2);

    // R ���� ����
    std_msgs::Int32 r_msg;
    r_msg.data = R;
    pub_r.publish(r_msg);

    // ���� 1 ��ġ�� ����
    std_msgs::Int32 theta1_msg;
    theta1_msg.data = current_position1;
    pub_theta1.publish(theta1_msg);

    // ���� 2 ��ġ�� ����
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
  ros::init(argc, argv, "jammer_unit_node");
  DynamixelControl jammer_unit;
  ros::spin();
  return 0;
}
