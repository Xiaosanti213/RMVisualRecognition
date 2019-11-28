# RMVisualRecognition
Visual Recognition and Auto Aiming for BH RM Team 2020

## Prerequisites
* [NVIDIA Jetson TX2](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-tx2/)
* [JetPack 3.3](https://developer.nvidia.com/embedded/jetpack-3_3 "Official Website")
* [MYNT 2100](https://mynt-eye-s-sdk.readthedocs.io/zh_CN/latest/src/sdk/contents.html "SDK")
* ROS kinect 1.12.14
* python3.5 
* pymavlink
* opencv2
    
## Code
    The Project consists of two Parts: One for object detection(C++), the other for deciding next move(Python).
    ROS is used for communication.
------
- [ ] detect_ros_ws
    - [ ] build
    - [ ] devel
    - [ ] src
        - [ ] target_detect_package
            - [ ] include
            - [x] src
                - [x] target_detect.cpp
            - [x] CMakeLists.txt
            - [ ] package.xml
                
                
                
- [x] py_move_decision
    - [x] pymav_move_decicion.py
    
## Configuration

### Flash Jetpack 3.3

Internal OS Ubuntu16.04

### Install ROS & Configure WorkSpace

ROS package configuration steps lists Below. 
```
$ mkdir -p ~/detect_ros_ws/src
$ cd ~/detect_ros_ws/src
$ catkin_init_workspace

$ catkin_create_pkg target_detect_package std_msgs roscpp
$ rosed CMakeLists.txt # modify code
$ cd target_detect_package/src

$ cd ~/detect_ros_ws #~/detect_ros_ws
$ catkin_make

$ roscore #new terminal

$ cd ~/detect_ros_ws
$ source devel/setup.bash
$ rosrun target_detect
```

### Python3.5 etc

run directly
```
python pymav_move_decicion.py
```
pip install lacking libs

## Usage








