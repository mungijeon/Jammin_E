# Jamming-Project

### RF Module

### FPGA

### Manipulator
2-Axis Manipulator<br/>
ROS<br/>
Dynamixel

<Jammer_Unit><br/>
use after catkin_make<br/>
roslaunch jammer_unit jammer_control.launch<br/>
rostopic pub /dynamixel_positions std_msgs/Int32MultiArray "data: [ID1 position,ID2 position]"<br/>
-> degree(360`) = 0.088 * value(0~4095)<br/>
-> positon(0~4095) = 11.38 * degree(360`)<br/>

#### 24-04-06
git test
