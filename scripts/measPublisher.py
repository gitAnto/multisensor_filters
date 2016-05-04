#!/usr/bin/env python

import rospy
from multisensor_filters.msg import PointWithCovarianceStamped

import random

def talker():
    pub = rospy.Publisher('topic', PointWithCovarianceStamped, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        msg = PointWithCovarianceStamped()

        msg.point.point.x = random.uniform(1,10)
        msg.point.point.y = random.uniform(1,10)
        msg.point.covariance = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]

        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass