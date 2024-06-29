## 프로젝트 초기 설정
## CMake의 최소 버전 설정
cmake_minimum_required(VERSION 3.0.2)

## 프로젝트 이름 설정
# 해당 이름으로 catkin_ws/devel/lib 경로에 폴더가 생성된다
# package.xml에서 package 이름과 동일하게 해주어야 한다
# launch 파일에서 package 이름과 동일하게 해주어야 한다
project(jammer_unit)

## 필요한 패키지 찾기
find_package(catkin REQUIRED COMPONENTS
  dynamixel_workbench_msgs
  roscpp
  std_msgs
)

## catkin 패키지 설정
# 이게 있어야지 catkin_ws/devel/lib 경로에 실행 파일이 생성된다
catkin_package(
)

## 헤더 파일 디렉토리 포함
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## C++ 실행 파일 생성
# 상대경로에 해당하는 소스 코드를
# 설정한 실행 파일의 이름으로 생성하겠다
# 여기서 실행 파일이란 rosrun 명령에서 사용되는 실행파일 이름이다
# 상대경로에 해당하는 소스 코드에의 ros::init에서 설정한 노드이름과 동일하게 설정하면 좋다
# launch 파일에서도 실행 파일 이름을 동일하게 설정해준다
add_executable(jammer_unit_node src/jammer_unit_node.cpp)

## 라이브러리 링크
# 실행 파일에 링크될 라이브러리를 지정한다
# add_executable 함수에서 설정한 실행 파일의 이름과 같아야한다
target_link_libraries(jammer_unit_node
  ${catkin_LIBRARIES}
)
