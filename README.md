# hakuroukun_ws
## Code for Hakuroukun Cleaning Robot

### Setting up the enviroment

-----

* **Enable GUI within Docker containers**

  > **! Caution:** This method exposes PC to external source. Therefore, a more secure alternative way is expected for using GUI within Docker containers. This problem was raised in [Using GUI's with Docker](https://wiki.ros.org/es/docker/Tutorials/GUI#:~:text=%2D%2Dpulse.-,Using%20X%20server,-X%20server%20is)

```bash
#This command is required to run every time the PC is restarted
xhost + 
```
Make a X authentication file with proper permissions for the container to use.

```bash
# If not working, try to run "sudo rm -rf /tmp/.docker.xauth" first
cd ./src/hakuroukun_dockerfiles/
chmod +x ./install/xauth.sh && ./install/xauth.sh
```

### With pure pursuit controller 
To run gazebo with ekf localization 
```
docker exec -it hakuroukun-robot bash 
roslaunch hakuroukun_launch bringup.launch
```

On another terminal, run pure pursuit controller 
```
docker exec -it hakuroukun-robot bash 
roslaunch hakuroukun_control hakuroukun_control.launch
```

### Experiment : To run robot with any controller:

0. Check ```cat /dev/ttyACM*``` for GPS, Arduino, IMU

1. Sensor bringup :
    - need to calculate and update : ``` <param name="~rotation_angle" value="-90"/> ```
    ```
    docker exec -it hakuroukun-robot bash 
    roslaunch hakuroukun_launch sensor_bringup.launch
    ```

2. Arduino firmware :
    - Upload `${hakuroukun_communication}/firmware/motor_control/motor_control.ino` to Arduino with Arduino IDE.

3. Communication bringup :
    ```
    docker exec -it hakuroukun-robot bash
    roslaunch hakuroukun_launch communication_bringup.launch
    ```

4. Controller execution : 
    ```
    docker exec -it hakuroukun-robot bash
    roslaunch hakuroukun_launch controller.launch
    ```