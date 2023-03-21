nmea_comms
==========

Generalized ROS interface to NMEA-speaking devices. 

Provides a serial node and socket server node, which relay CRLF-terminated strings on `rx` and `tx`
ROS topics using the [nmea_msgs/Sentence](http://docs.ros.org/latest-available/api/nmea_msgs/html/msg/Sentence.html) message type.

# Launch

~~~
source $HOME/catkin_ws/devel/setup.bash
roslaunch nmea_comms f9p_nmea_sentence.launch
~~~

With this launch, `/f9p/nmea_sentence (nmea_msgs/Sentence)` is published.  
The topic will be published even indoors where GNSS signals cannot reach.

# Parameter description

The parameters are set in `launch/f9p_nmea_sentence.launch` .

|Name|Type|Description|Default value|
|:---|:---|:---|:---|
|port|string|The serial device port to be used|/dev/ttyUSB0|
|baud|double|Baudrate|115200|

# Related packages
- [nmea2fix](https://github.com/MapIV/nmea2fix)