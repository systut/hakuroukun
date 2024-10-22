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