#include "ros/ros.h"
#include "dynamixel_workbench_msgs/DynamixelCommand.h"
// dynamixel_workbench_msgs::DynamixelStateList 메세지 타입을 사용하기 위함
// 이는 다이나믹셀 상태 정보를 포함
#include "dynamixel_workbench_msgs/DynamixelStateList.h"
#include "std_msgs/Int32.h"       // 정수 R 값을 위한 메시지 타입

class DynamixelControl
{
public:
  DynamixelControl()
  {
    // 서비스 클라이언트 생성: dynamixel_command 서비스를 호출하기 위한 클라이언트를 생성
    client = nh_.serviceClient<dynamixel_workbench_msgs::DynamixelCommand>("/dynamixel_workbench/dynamixel_command");
    // 구독자 생성: dynamixel_state 토픽을 구독하고, 콜백 함수를 설정
    sub = nh_.subscribe("/dynamixel_workbench/dynamixel_state", 10, &DynamixelControl::Callback, this);
    // topic1을 통해 구독
    sub_r = nh_.subscribe("/rf_layer", 10, &DynamixelControl::RCallback, this);    
    // topic2를 통해 발행
    pub_theta1 = nh_.advertise<std_msgs::Int32>("/topic_theta1", 10);
    pub_theta2 = nh_.advertise<std_msgs::Int32>("/topic_theta2", 10);
    pub_r = nh_.advertise<std_msgs::Int32>("/topic_r", 10);

    ros::Duration(0.5).sleep();
    setSpeed(1,40);
    setSpeed(2,300);
    setPosition(1, 1000);
    //setPosition(2, 0);
    //모터 2의 각도를 0, 90도로 설정하면 90도 일때, z축 위에 드론이 존재한다 가정하므로 theta1의 변화가 무의미해진다.
    //따라서 test하기에 부적절한 수치라 생각해 바꿔주었다.
    setPosition(2, 341);
  }

  // 콜백 함수 정의: dynamixel_state 메세지를 처리
  void Callback(const dynamixel_workbench_msgs::DynamixelStateList::ConstPtr& msg)
  {
    // dynamixel_state 리스트 내의 각 상태 메세지를 반복 처리
    for (const auto& state : msg->dynamixel_state)
    {
      if (state.id == 1)
      {
        current_position1 = state.present_position;
        //ROS_INFO("Current Position of ID %d: %d", state.id, current_position1);
        if (current_position1 <= 1024)
        {
          setPosition(1, 3100);
          //setPosition(2,1024);
          setPosition(2,682);
        }
        else if (current_position1 >= 3072)
        {
          setPosition(1,1000);
          //setPosition(2,0);
          setPosition(2,341);
        }
      }
      else if (state.id == 2)
      {
        current_position2 = state.present_position;
      }
    }
  }

  // R 값을 받는 콜백 함수
  void RCallback(const std_msgs::Int32::ConstPtr& msg)
  {
    int R = msg->data;
    ROS_INFO("R: %d", R);
    ROS_INFO("theta1: %d", current_position1);
    ROS_INFO("theta2: %d", current_position2);

    // R 값을 발행
    std_msgs::Int32 r_msg;
    r_msg.data = R;
    pub_r.publish(r_msg);

    // 모터 1 위치를 발행
    std_msgs::Int32 theta1_msg;
    theta1_msg.data = current_position1;
    pub_theta1.publish(theta1_msg);

    // 모터 2 위치를 발행
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
