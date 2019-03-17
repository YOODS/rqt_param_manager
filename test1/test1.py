#!/usr/bin/python

import numpy as np
import roslib
import rospy
from std_msgs.msg import String


def cb_param(str):
  a=eval(str.data)
  for key in a:
    if key is 'zcrop':
      print "zcrop param",a[key]
    elif key is 'sphare':
      print "sphare param",a[key]
  return

###############################################################
rospy.init_node('test1',anonymous=True)
sb_param=rospy.Subscriber('/test1/param',String,cb_param)

try:
  rospy.spin()
except KeyboardInterrupt:
  print "Shutting down"
