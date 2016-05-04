#!/usr/bin/env python
#*********************************************************************
#*  MIT License
#*
#*  Copyright (c) 2015 Antonio Petitti
#*
#*  Permission is hereby granted, free of charge, to any person obtaining a copy
#*  of this software and associated documentation files (the "Software"), to deal
#*  in the Software without restriction, including without limitation the rights
#*  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#*  copies of the Software, and to permit persons to whom the Software is
#*  furnished to do so, subject to the following conditions:
#*  
#*  The above copyright notice and this permission notice shall be included in all
#*  copies or substantial portions of the Software.
#*  
#*  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#*  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#*  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#*  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#*  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#*  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
#*  SOFTWARE.
#*********************************************************************

PKG = 'multisensor_filters'
NAME = 'filter_node_kf'

import roslib; roslib.load_manifest(PKG)
import rospy

import numpy as np

from geometry_msgs.msg import PointStamped

from common.filter_node import FilterNode
from filters.filter_kf import FilterKF

from math import pow


class FilterNodeKF(FilterNode):
		
	def __init__(self, A, H, P, Q, filterRate):
		FilterNode.__init__(self)
		self.kf = FilterKF(A, H, P, Q)

		self.filterRate = filterRate

	def runFilter(self):

		r = rospy.Rate(self.filterRate) 
		while not rospy.is_shutdown():
			if self.flag_reset:
				self.kf.reset(self.getReset())
				self.flag_reset = 0

			self.kf.iteration(self.getMeasures())

			self.pose_pub_.publish(self.kf.getState())
			
			person_point = PointStamped()
			person_point.header = self.kf.getState().header
			person_point.header.stamp = rospy.Time.now()
			person_point.point = self.kf.getState().pose.position
			self.point_pub_.publish(person_point)

			self.tf_person.sendTransform((self.kf.getState().pose.position.x,self.kf.getState().pose.position.y,0),
                     		(self.kf.getState().pose.orientation.x,self.kf.getState().pose.orientation.y,self.kf.getState().pose.orientation.z,self.kf.getState().pose.orientation.w),
                     		rospy.Time.now(),
                     		"person_link",
                     		self.kf.getState().header.frame_id)

			r.sleep()

def filter_main(argv=None):
	rospy.init_node(NAME, anonymous=False)

	filterRate = rospy.get_param('filterRate',5)
	dt = 1.0/filterRate # 1/frequency
	sigma_0 = rospy.get_param('sigma_0',5.0)
	sigma_w = pow(dt*sigma_0,2)

	A_str = rospy.get_param('A')
	A = np.matrix(A_str)

	H_str = rospy.get_param('H')
	H = np.matrix(H_str)
	
	P_str = rospy.get_param('P')
	P = np.matrix(P_str)
	
	Q_str = rospy.get_param('Q')
	Q = np.matrix(Q_str)

	#TO-DO: add safety control on matrix dimensions

	filter_ = FilterNodeKF(A, H, P, Q, filterRate)
	filter_.runFilter()

if __name__ == '__main__':
	filter_main()
