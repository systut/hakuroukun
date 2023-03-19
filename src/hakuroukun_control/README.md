# 概要
This node is the driver node used to control hakuroukun robot via serial connection

## Launch the node
roslaunch hakuroukun_control hakuroukun_control.launch

## Use in launch file
<include file="$(find hakuroukun_control)/launch/hakuroukun_control.launch">
    <arg name="port" value="$(arg port)"/>
    <arg name="baud_rate" value="$(arg baud_rate)"/>
    <arg name="controller_rate" value="$(arg controller_rate)"/>
</include>


# Topics
## Subscribed Topics
Subscribing the command velocity from planner tracking controller
/cmd_vel(geometry_msgs/Twist) 


