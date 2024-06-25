// ROS�� �ٽ� ����� ����ϱ� ���� �ʿ�
#include "ros/ros.h"
// ROS ǥ�� �޽��� Ÿ���� 'std_msgs' ��Ű������ 'Int32MultiArray' �޽��� Ÿ���� ���Խ�Ű�� ���� ��� ������ ����
// 'Int32MultiArrayLayout'�� ���� ���� 32��Ʈ ����('int32')�� �迭 ���·� �����ϴ� �޽��� Ÿ���̴�.
// ���� ���� 32��Ʈ ������ �迭 ���·� �����ϴ� �� ���
#include "std_msgs/Int32MultiArray.h"
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
    command_client_ = nh_.serviceClient<dynamixel_workbench_msgs::DynamixelCommand>("/dynamixel_workbench/dynamixel_command");
    // ������ �ʱ�ȭ: dynamixel_positions ������ �����ϰ� �ݹ� �Լ� ����
    sub_ = nh_.subscribe("dynamixel_positions", 10, &DynamixelControl::commandCallback, this);
  }

  // �ݹ� �Լ�: dynamixel_positions ���ȿ��� �޽����� ���� �� ȣ��
  void commandCallback(const std_msgs::Int32MultiArray::ConstPtr& msg)
  {
    // ù ��° ������ ��ġ ����
    setPosition(1, msg->data[0]);
    // �� ��° ������ ��ġ ����
    setPosition(2, msg->data[1]);
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
    command_client_.call(srv);
  }

// private ����� ���� Ŭ������ �ٸ� ����� ������ �� �ִ� ����� �����.
// �Ϲ������� ��� ������ ������� �ϰ�, ��� �Լ��� �����ϴ� ���� �Ϲ����̴�.
private:
  // ROS ��� �ڵ鷯
  ros::NodeHandle nh_;
  // ���� Ŭ���̾�Ʈ
  ros::ServiceClient command_client_;
  // ������
  ros::Subscriber sub_;
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