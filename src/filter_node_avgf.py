#!/usr/bin/env python
#*********************************************************************
#*  MIT License
#*
#*  Copyright (c) 2015 Donato Di Paola
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
NAME = 'filter_node_avgf'

import roslib; roslib.load_manifest(PKG)
import rospy

from geometry_msgs.msg import PointStamped

from common.filter_node import FilterNode
from filters.filter_avgf import FilterAVG

class FilterNodeAVG(FilterNode):
		
	def __init__(self):
		FilterNode.__init__(self)
		self.avgf = FilterAVG()

	def runFilter(self):

		r = rospy.Rate(5)
		while not rospy.is_shutdown():
			self.avgf.iteration(self.getMeasures())
			self.pose_pub_.publish(self.avgf.getState())
			
			person_point = PointStamped()
			person_point.header = self.avgf.getState().header
			person_point.point = self.avgf.getState().pose.position
			self.point_pub_.publish(person_point)
			r.sleep()

def filter_main(argv=None):
	rospy.init_node(NAME, anonymous=False)
	filter_ = FilterNodeAVG()
	filter_.runFilter()

if __name__ == '__main__':
	filter_main()
