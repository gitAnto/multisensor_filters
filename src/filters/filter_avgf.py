#!/usr/bin/env python
#*********************************************************************
#*  MIT License
#*
#*  Copyright (c) 2015 Donato di Paola and Antonio Petitti
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
NAME = 'filter_avgf'

import roslib; roslib.load_manifest(PKG)
import rospy

from multisensor_filters.msg import PointWithCovarianceStamped
from common._filter import Filter

class FilterAVG(Filter):

	def iteration(self, measures):
		
		x_tmp = 0.0
		y_tmp = 0.0
		
		weight_sum = 0.0

		if measures: 

			for measure in measures:
				if measure.point.covariance[0] !=0  or measure.point.covariance[3] !=0:
					weight_sum = weight_sum + 1 / (measure.point.covariance[0] + measure.point.covariance[3]) 

			for measure in measures:
				weight = 1.0

				if measure.point.covariance[0] !=0  or measure.point.covariance[3] !=0:
					weight = 1 / (measure.point.covariance[0] + measure.point.covariance[3]) 
					weight = weight / weight_sum
				
				x_tmp = x_tmp + (weight * measure.point.point.x)
				y_tmp = y_tmp + (weight * measure.point.point.y)

			self.state.pose.position.x = x_tmp 
			self.state.pose.position.y = y_tmp 

