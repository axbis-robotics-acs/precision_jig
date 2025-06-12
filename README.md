# Precision_Jig


# Build
### *This Package is implemented at ROS2-Humble.*

```
### I assume that you have installed the ros-humble-desktop package using the apt-get command.
### I recommand the /home/<user_home>/precision_jig_ws/src

$ mkdir -p ~/precision_jig_ws/src
$ cd ~/precision_jig_ws/src
$ git clone https://github.com/axbis-robotics-acs/precision_jig.git

$ sudo apt-get install ros-humble-realsense2*

$ cd ~/precision_jig_ws
$ colcon build
$ . install/setup.bash
$ source precision_jig_ws/install/local_setup.bash

### Recommended
# $ echo 'source ~/precision_jig_ws/install/local_setup.bash' >> ~/.bashrc 
```

# Launch
```
$ ros2 launch precision_jig amr_vision_check.launch.py 
```