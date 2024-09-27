# Jamming-Project

### RF Module

### FPGA

### Manipulator

#### [Package]
DynamixelSDK, dynamixel-workbench, dynamixel-workbench-msgs<br/>
Radar_Unit, Jammer_Unit<br/><br/>
$ cd ~/catkin_ws/src<br/>
$ git clone https://github.com/ROBOTIS-GIT/dynamixel-workbench.git<br/>
$ git clone https://github.com/ROBOTIS-GIT/dynamixel-workbench-msgs.git<br/>
$ git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git<br/>
$ cd ~/catkin_ws<br/>
$ catkin_make<br/>

workbench -> config 파일 수정
simulink 에서 topic 이름 지정이 아닌 직접 타이핑 해줌

#### [Radar_Unit]
roslaunch radar_unit radar_control.launch<br/>
-> velocity(0-32767): 0.229[rev/min] <br/>
-> position(0-4095)
#### [Jammer_Unit]
roslaunch jammer_unit jammer_control.launch<br/>
rostopic pub /dynamixel_positions std_msgs/Int32MultiArray "data: [ID1 position,ID2 position]"<br/>
-> degree(360) = 0.088 * value(0-4095) <br/>
-> positon(0-4095) = 11.38 * degree(360)<br/>

### [MATLAB]
$ matlab
>> rosinit

Simulink -> Run
