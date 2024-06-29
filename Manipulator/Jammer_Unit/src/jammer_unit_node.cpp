// ROS의 핵심 기능을 사용하기 위해 필요
#include "ros/ros.h"
// ROS 표준 메시지 타입인 'std_msgs' 패키지에서 'Int32MultiArray' 메시지 타입을 포함시키기 위한 헤더 파일을 포함
// 'Int32MultiArrayLayout'는 여러 개의 32비트 정수('int32')를 배열 형태로 포함하는 메시지 타입이다.
// 여러 개의 32비트 정수를 배열 형태로 전달하는 데 사용
#include "std_msgs/Int32.h"
// DynamixelCommand를 사용하기 위함
#include "dynamixel_workbench_msgs/DynamixelCommand.h"

// DynamixelControl 클래스 선언
class DynamixelControl
{
// public 멤버는 구조체 또는 클래스 외부에서 접근할 수 있는 구조체 또는 클래스의 공개 멤버다.
public:
  // 생성자: 초기화 및 ROS 서비스 클라이언트와 구독자를 설정
  DynamixelControl()
  {
    // 서비스 클라이언트 초기화: /dynamixel_workbench/dynamixel_command 서비스와 통신
    client = nh_.serviceClient<dynamixel_workbench_msgs::DynamixelCommand>("/dynamixel_workbench/dynamixel_command");
    // 구독자 초기화: dynamixel_positions 토픽을 구독하고 콜백 함수 설정
    //sub_theta1 = nh_.subscribe("position_1", 10, &DynamixelControl::commandCallback1, this);
    //sub_theta2 = nh_.subscribe("position_2", 10, &DynamixelControl::commandCallback2, this);
    sub_theta1 = nh_.subscribe("topic_theta1", 10, &DynamixelControl::commandCallback1, this);
    sub_theta2 = nh_.subscribe("topic_theta2", 10, &DynamixelControl::commandCallback2, this);
  }

  // 콜백 함수: dynamixel_positions 토픽에서 메시지를 받을 때 호출
  void commandCallback1(const std_msgs::Int32::ConstPtr& msg)
  {
    setPosition(3, msg->data);
  }
  void commandCallback2(const std_msgs::Int32::ConstPtr& msg)
  {
    setPosition(4, msg->data);
  }
  
  // 모터 위치 설정 함수
  void setPosition(int id, int position)
  {
    // DynamixelCommand 서비스 요청 메시지 생성
    dynamixel_workbench_msgs::DynamixelCommand srv;
    // 요청 명령을 빈 문자열로 설정
    srv.request.command = "";
    // 요청 모터 ID 설정
    srv.request.id = id;
    // 요청 주소 이름 설정: Goal_Position (목표 위치)
    srv.request.addr_name = "Goal_Position";
    // 요청 값 설정: 위치
    srv.request.value = position;

    // 서비스를 호출하여 요청을 보냄
    client.call(srv);
  }

// private 멤버는 오직 클래스의 다른 멤버만 접근할 수 있는 비공개 멤버다.
// 일반적으로 멤버 변수는 비공개로 하고, 멤버 함수는 공개하는 것이 일반적이다.
private:
  // ROS 노드 핸들러
  ros::NodeHandle nh_;
  // 서비스 클라이언트
  ros::ServiceClient client;
  // 구독자
  ros::Subscriber sub_theta1;
  ros::Subscriber sub_theta2;
};

// 메인 함수
int main(int argc, char **argv)
{
  // ROS 노드 초기화
  ros::init(argc, argv, "jammer_unit_node");
  // DynamixelControl 객체 생성
  DynamixelControl jammer_unit;
  // ROS 이벤트 루프 시작: 콜백 함수가 호출될 수 있도록 함
  ros::spin();
  return 0;
}