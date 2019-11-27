# RMVisualRecognition
Visual Recognition and Auto Aiming for BH RM Team 2020

## Prerequisites
    NVIDIA Jetson TX2
    JetPack 3.3 (Ubuntu16.04)
    [MYNT 2100](https://mynt-eye-s-sdk.readthedocs.io/zh_CN/latest/src/sdk/install_windows_exe.html "SDK")
    ROS kinect 1.12.14
    python3.5 pymavlink
    ...
    
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
    








