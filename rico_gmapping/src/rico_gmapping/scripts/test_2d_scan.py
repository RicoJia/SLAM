#!/usr/bin/env python
"""
Theory: 
    - Shbang is a MUST, must use python like above
    - Latched Topic: for slow changing topics. Msgs will be saved and sent to incoming connections. 
    If there's multiple pubs, only the last one gets sent, not from each pub. 
"""
import rospy
from sensor_msgs.msg import LaserScan


if __name__ == '__main__':
    rospy.init_node("test_2d_scan", anonymous=True)     #anonymous=true ensures unique node name by adding random numbers
    # private namespace name 
    # pub = rospy.Publisher("~test_scan_before", LaserScan, latch=True, queue_size=1)
    pub = rospy.Publisher("test_scan_before", LaserScan, latch=True, queue_size=1)
    rospy.spin()
