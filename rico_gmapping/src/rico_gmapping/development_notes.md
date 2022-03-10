========================================================================
## Build An Environment 
========================================================================
1. [gmapping with ros bag](http://wiki.ros.org/slam_gmapping/Tutorials/MappingFromLoggedData)
    ```bash
    # ros gmap:
    rosrun gmapping slam_gmapping scan:=base_scan
    rosbag play BAG_NAME
    ```

