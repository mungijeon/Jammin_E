#include "ros/ros.h"
#include "dynamixel_workbench_msgs/DynamixelCommand.h"
// dynamixel_workbench_msgs::DynamixelStateList 메세지 타입을 사용하기 위함
// 이는 다이나믹셀 상태 정보를 포함
#include "dynamixel_workbench_msgs/DynamixelStateList.h"

class DynamixelControl
{
public:
  DynamixelControl()
  {
    // 서비스 클라이언트 생성: dynamixel_command 서비스를 호출하기 위한 클라이언트를 생성
    client = nh_.serviceClient<dynamixel_workbench_msgs::DynamixelCommand>("/dynamixel_workbench/dynamixel_command");
    // 구독자 생성: dynamixel_state 토픽을 구독하고, 콜백 함수를 설정
    sub = nh_.subscribe("/dynamixel_workbench/dynamixel_state", 10, &DynamixelControl::Callback, this);

    ros::Duration(0.5).sleep();
    setSpeed(1,40);
    setSpeed(2,300);
    setPosition(1, 1000);
    setPosition(2, 0);
  }

  // 콜백 함수 정의: dynamixel_state 메세지를 처리
  void Callback(const dynamixel_workbench_msgs::DynamixelStateList::ConstPtr& msg)
  {
    // dynamixel_state 리스트 내의 각 상태 메세지를 반복 처리
    for (const auto& state : msg->dynamixel_state)
    {
      if (state.id == 1)
      {
        int current_position = state.present_position;
        ROS_INFO("Current Position of ID %d: %d", state.id, current_position);
        if (current_position <= 1024)
        {
          setPosition(1, 3100);
          setPosition(2,1024);
        }
        else if (current_position >= 3072)
        {
          setPosition(1,1000);
          setPosition(2,0);
        }
      }
    }
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
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "jammer_unit_node");
  DynamixelControl jammer_unit;
  ros::spin();
  return 0;
}
