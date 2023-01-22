#!/usr/bin/env python
import sys
import time
import numpy as np
import rospy
from sensor_msgs.msg import  LaserScan

firstScan = True
lastScan = False
t0 = time.time()
count = 0
def update_points(msg):
   global fd, firstScan, t0, count, lastScan
   if firstScan:
      t0 = time.time()
      firstScan = False
   count += 1
   t1 = time.time()
   sz = len(msg.ranges)
   fd.write(str(t1-t0))
   fd.write(' ')
   fd.write(str(sz))
   fd.write(' ')
   for ii in range(0, sz):
      fd.write(str(msg.ranges[ii]))
      fd.write(' ')
   fd.write('\n')
   if count == 1000:
      lastScan = True
   print(count)

if __name__ == '__main__':
   global fd
   fd = open(sys.argv[1],'w')
   rospy.init_node('saveScans')
   subs = rospy.Subscriber('/scan', LaserScan, update_points)
   rate = rospy.Rate(10)
   while not rospy.is_shutdown() and not lastScan:
      rate.sleep()
