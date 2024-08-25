# Jamming-Project

### RF Module

### FPGA

### Manipulator
package DynamixelSDK, dynamixel-workbench, dynamixel-workbench-msgs<br/>
$ cd ~/catkin_ws/src<br/>
$ git clone https://github.com/ROBOTIS-GIT/dynamixel-workbench.git<br/>
$ git clone https://github.com/ROBOTIS-GIT/dynamixel-workbench-msgs.git<br/>
$ git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git<br/>
$ cd ~/catkin_ws<br/>
$ catkin_make<br/>
#### <Radar_Unit>
roslaunch radar_unit radar_control.launch<br/>
-> velocity(0-32767): 0.229[rev/min] <br/>
-> position(0-4095)
#### <Jammer_Unit>
roslaunch jammer_unit jammer_control.launch<br/>
rostopic pub /dynamixel_positions std_msgs/Int32MultiArray "data: [ID1 position,ID2 position]"<br/>
-> degree(360) = 0.088 * value(0-4095) <br/>
-> positon(0-4095) = 11.38 * degree(360)<br/>
