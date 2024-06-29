// ROS�� �ٽ� ����� ����ϱ� ���� �ʿ�
#include "ros/ros.h"
// ROS ǥ�� �޽��� Ÿ���� 'std_msgs' ��Ű������ 'Int32MultiArray' �޽��� Ÿ���� ���Խ�Ű�� ���� ��� ������ ����
// 'Int32MultiArrayLayout'�� ���� ���� 32��Ʈ ����('int32')�� �迭 ���·� �����ϴ� �޽��� Ÿ���̴�.
// ���� ���� 32��Ʈ ������ �迭 ���·� �����ϴ� �� ���
#include "std_msgs/Int32.h"
// DynamixelCommand�� ����ϱ� ����
#include "dynamixel_workbench_msgs/DynamixelCommand.h"

// DynamixelControl Ŭ���� ����
class DynamixelControl
{
// public ����� ����ü �Ǵ� Ŭ���� �ܺο��� ������ �� �ִ� ����ü �Ǵ� Ŭ������ ���� �����.
public:
  // ������: �ʱ�ȭ �� ROS ���� Ŭ���̾�Ʈ�� �����ڸ� ����
  DynamixelControl()
  {
    // ���� Ŭ���̾�Ʈ �ʱ�ȭ: /dynamixel_workbench/dynamixel_command ���񽺿� ���
    client = nh_.serviceClient<dynamixel_workbench_msgs::DynamixelCommand>("/dynamixel_workbench/dynamixel_command");
    // ������ �ʱ�ȭ: dynamixel_positions ������ �����ϰ� �ݹ� �Լ� ����
    //sub_theta1 = nh_.subscribe("position_1", 10, &DynamixelControl::commandCallback1, this);
    //sub_theta2 = nh_.subscribe("position_2", 10, &DynamixelControl::commandCallback2, this);
    sub_theta1 = nh_.subscribe("topic_theta1", 10, &DynamixelControl::commandCallback1, this);
    sub_theta2 = nh_.subscribe("topic_theta2", 10, &DynamixelControl::commandCallback2, this);
  }

  // �ݹ� �Լ�: dynamixel_positions ���ȿ��� �޽����� ���� �� ȣ��
  void commandCallback1(const std_msgs::Int32::ConstPtr& msg)
  {
    setPosition(3, msg->data);
  }
  void commandCallback2(const std_msgs::Int32::ConstPtr& msg)
  {
    setPosition(4, msg->data);
  }
  
  // ���� ��ġ ���� �Լ�
  void setPosition(int id, int position)
  {
    // DynamixelCommand ���� ��û �޽��� ����
    dynamixel_workbench_msgs::DynamixelCommand srv;
    // ��û ����� �� ���ڿ��� ����
    srv.request.command = "";
    // ��û ���� ID ����
    srv.request.id = id;
    // ��û �ּ� �̸� ����: Goal_Position (��ǥ ��ġ)
    srv.request.addr_name = "Goal_Position";
    // ��û �� ����: ��ġ
    srv.request.value = position;

    // ���񽺸� ȣ���Ͽ� ��û�� ����
    client.call(srv);
  }

// private ����� ���� Ŭ������ �ٸ� ����� ������ �� �ִ� ����� �����.
// �Ϲ������� ��� ������ ������� �ϰ�, ��� �Լ��� �����ϴ� ���� �Ϲ����̴�.
private:
  // ROS ��� �ڵ鷯
  ros::NodeHandle nh_;
  // ���� Ŭ���̾�Ʈ
  ros::ServiceClient client;
  // ������
  ros::Subscriber sub_theta1;
  ros::Subscriber sub_theta2;
};

// ���� �Լ�
int main(int argc, char **argv)
{
  // ROS ��� �ʱ�ȭ
  ros::init(argc, argv, "jammer_unit_node");
  // DynamixelControl ��ü ����
  DynamixelControl jammer_unit;
  // ROS �̺�Ʈ ���� ����: �ݹ� �Լ��� ȣ��� �� �ֵ��� ��
  ros::spin();
  return 0;
}