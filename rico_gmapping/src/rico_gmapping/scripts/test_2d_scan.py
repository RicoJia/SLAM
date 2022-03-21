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
    msg = LaserScan()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "base_scan"
    msg.angle_min = -3.14
    msg.angle_max = 3.14
    msg.angle_increment = (msg.angle_max - msg.angle_min) / 360
    msg.range_min = 0.0
    msg.range_max = 3.0
    msg.ranges.extend([0.9]* 360)
    pub.publish(msg)
    rospy.spin()
