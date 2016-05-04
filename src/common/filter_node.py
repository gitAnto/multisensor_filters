#!/usr/bin/env python
#*********************************************************************
#*  MIT License
#*
#*  Copyright (c) 2015 Donato Di Paola, Antonio Petitti
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
NAME = 'filter_node'

import roslib; roslib.load_manifest(PKG)
import rospy

import tf

from geometry_msgs.msg import PoseStamped, PointStamped, Point
from multisensor_filters.msg import PointWithCovarianceStamped

class FilterNode:
	def __init__(self):
		
		topics_name = rospy.get_param('measTopics')
		self.n_meas = len(topics_name)

		self.measures = []
		self.flagMeas = []
		self.subs     = []
		self.topics   = {}

		i = 0
		for topic in topics_name:
			self.measures.append(PointWithCovarianceStamped())
			self.flagMeas.append(0)
			self.subs.append(rospy.Subscriber(topic, PointWithCovarianceStamped, self.callback, topic))
			self.topics[topic] = i

			i = i + 1

		self.reset = Point()
		self.tf_person = tf.TransformBroadcaster()		
		self.flag_reset = 0
		
		rospy.Subscriber("/output_point_reset", PoseStamped, self.callback_reset)	

		self.pose_pub_ = rospy.Publisher("/pose", PoseStamped, queue_size=10)
		self.point_pub_ = rospy.Publisher("/point", PointStamped, queue_size=10)

	def callback(self, measurement, topic_name):
		self.measures[self.topics[topic_name]] = measurement
		self.flagMeas[self.topics[topic_name]] = 1


	def callback_reset(self, person_pose):
		self.reset.x = person_pose.pose.position.x
		self.reset.y = person_pose.pose.position.y
		self.reset.z = person_pose.pose.position.z

		self.flag_reset = 1

	def getMeasures(self):		
		measures = self.measures	

		return measures

	def getReset(self):
		return self.person_reset

	def runFilter(self):
		print "virtual method" 

