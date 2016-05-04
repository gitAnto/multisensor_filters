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
NAME = 'filter'

import roslib; roslib.load_manifest(PKG)
import rospy

from geometry_msgs.msg import PoseStamped

class Filter:
	def __init__(self):

		self.state = PoseStamped()
		self.state.header.frame_id = "/map"

	def iteration(self):
		print "virtual method"

	def getState(self):
		return self.state
