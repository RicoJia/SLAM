# rico_gmapping
## Introduction - Note: rico_gmapping is currently not complete
1. Description
    - Motivation: this is a custom gmapping package based on [ROS 2D SLAM gmapping](https://github.com/ros-perception/slam_gmapping)
    - In this repo, we are using a minimal example of 2D robot navigation as the testing environment. [see code](https://github.com/RicoJia/3D_Motion_Planning-/tree/master/2d_planning_playground/src/costmap_plugins) 
    - [Gmapping page](http://wiki.ros.org/gmapping)
    - Gmapping flowchart 
        <p align="center">
        <img src="https://user-images.githubusercontent.com/39393023/164169796-9291c6df-0d33-47b8-9175-24db8a5da1b3.png""" height="400" width="width"/>
        </p>

2. Intended changes
    1. For scan-matching, instead of a simplistic 8 direction search for robot transform between the current scan and last scan, we use: 
        - ICP - SVD method 
        - ICP - Gauss-Newton Method
        - PCL - ICP 
        - PCL - NDT (Normal Distribution Transform)
        
## Usage
1. Build and Run
    ```bash
    catkin_make
    source devel/setup.bash
    # 1. Run 2d scan test to see the ICP/PCL scan matching implementations
    roslaunch rico_gmapping rico_gmapping.launch test_2d_scan:=True

    # 2. launch gmapping with gazebo environment
    roslaunch rico_gmapping rico_gmapping.launch test_with_gazebo:=True
    ```
