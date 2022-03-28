========================================================================
## Build An Environment 
========================================================================
1. [gmapping with ros bag](http://wiki.ros.org/slam_gmapping/Tutorials/MappingFromLoggedData)
    ```bash
    # ros gmap:
    rosrun gmapping slam_gmapping scan:=base_scan
    rosbag play BAG_NAME
    ```

2. Frames: 
    - ```/map```: fixed world frame
    - ```/odom```: from odometer, subject to long term drift (/map vs /odom)
    - ```/base_link```: the center of the robot
    - ```/base_scan```: frame of the 2D Lidar

3. services: 
    1. ```dynamic_map```?

========================================================================
## ICP  
========================================================================
2. icp -GN


