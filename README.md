# 🚗 Hakuroukun Project

A modular robotics simulation environment with support for various path tracking algorithms like Pure Pursuit, Stanley, and Fuzzy Logic. Built with Docker and X11 forwarding support for both Windows and Ubuntu.

---

## 📚 Catalog

### 🛠️ Initial Setup
- [1 Windows](#1-windows)
  - [Install XLaunch](#install-xlaunch)
  - [Install Docker Desktop](#install-docker-desktop)
  - [Build Docker Images](#build-docker-images)
- [2 Ubuntu](#2-ubuntu)

### 🧪 Simulation
- [2.1 Pure Pursuit Control](#21-pure-pursuit-control)
- [2.2 Stanley Control](#22-stanley-control)
- [2.3 Fuzzy Logic Control](#23-fuzzy-logic-control)

---

# 🛠️ Initial Setup

## **1. Windows**

### Install XLaunch
- Download and install [VcXsrv Windows X Server](https://sourceforge.net/projects/vcxsrv/)
- Open **XLaunch** from the Start Menu

### Install Docker Desktop
- Install [Docker Desktop for Windows](https://docs.docker.com/desktop/setup/install/windows-install/)
- Launch **Docker Desktop**

### Build Docker Images
1.  Build Docker Image
    ```
    docker compose -f ./docker-compose.windows.yml up -d
    ```

2. Exec into running container
    ```
    docker exec -it hakuroukun-robot bash
    ```

3. Stop container
    ```
    docker compose -f ./docker-compose.windows.yml down
    ```

## **2. Ubuntu**
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


#### Build Docker Images
1.  Build Docker Image
    ```
    docker compose up -d
    ```

2. Exec into running container
    ```
    docker exec -it hakuroukun-robot bash
    ```

3. Stop container
    ```
    docker compose down
    ```

# Simulation : 
## Pure Pursuit controller 

1. Change mode to "simulation" in hakuroukun_launch/launch/bringup.launch
    ```
    <arg name="simulation" default="true" />
    ```
2. Run gazebo with ekf localization 
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

1. Change mode in hakuroukun_launch/launch/bringup.launch
    ```
    <arg name="simulation" default="false" />
    ```
2. GPS Rotation angle _ calibration :
    - need to calculate and update : ``` <param name="~rotation_angle" value="-90"/> ```
    - how measure : 
        - The robot in default position is heading in y axis
        - Run the robot and get coordinate from GPS by manual mode
        - Calculate the rotation angle by coordinate

3. Arduino firmware :
    - Upload `${hakuroukun_communication}/firmware/motor_control/motor_control.ino` to Arduino with Arduino IDE.

4. Bringup execution :
In the 1st terminal
    ```
    docker exec -it hakuroukun-robot bash
    roslaunch hakuroukun_launch bringup.launch
    ```

4. Controller execution : 
In the 2nd terminal
    ```
    docker exec -it hakuroukun-robot bash
    roslaunch hakuroukun_launch experiment_controller.launch
    ``` 