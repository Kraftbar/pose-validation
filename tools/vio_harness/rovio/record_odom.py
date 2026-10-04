#!/usr/bin/env python3
"""record_odom.py <out.tum> -- subscribe to /rovio/odometry (nav_msgs/Odometry, IMU pose in the world frame) and write TUM rows. Benchmark glue (ROS1 python)."""
import sys, rospy
from nav_msgs.msg import Odometry
f = open(sys.argv[1], 'w'); n = [0]
def cb(m):
    p, q = m.pose.pose.position, m.pose.pose.orientation
    f.write(f'{m.header.stamp.to_sec():.9f} {p.x} {p.y} {p.z} {q.x} {q.y} {q.z} {q.w}\n'); n[0] += 1
    if n[0] % 100 == 0: f.flush()
rospy.init_node('record_odom', anonymous=True); rospy.Subscriber('/rovio/odometry', Odometry, cb, queue_size=1000)
rospy.on_shutdown(lambda: (f.flush(), f.close())); rospy.spin()
