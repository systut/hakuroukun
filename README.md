# hakuroukun_ws
## Code for Hakuroukun Cleaning Robot


# With pure pursuit controller 
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