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
NAME = 'filter_kf'

import roslib; roslib.load_manifest(PKG)
import rospy

import numpy as np
import numpy.linalg as la

import tf

from multisensor_filters.msg import PointWithCovarianceStamped
from common._filter import Filter


class FilterKF(Filter):

	def __init__(self, A, H, P, Q):
		Filter.__init__(self)
		
		self.A = A 
		self.A_inv = la.inv(self.A)
		self.H = H
		self.P = P
		self.Q = Q
		self.n = A.shape[0]
		self.m = H.shape[0]
		self.x = np.zeros((self.n,1))
	
	def iteration(self, measures):
				
		y = np.linalg.inv(self.P)*self.x
		Y = la.inv(self.P)
         
		M = self.A_inv.T*Y*self.A_inv

		Sigma = M + la.inv(self.Q)
		Omega = M * la.inv(Sigma)

		# Prediction        
		y_ = (np.eye(np.shape(Omega)[1]) - Omega) * self.A_inv.T * y
		Y_ = M - Omega * Sigma * Omega.T

		# Estimate
		i = np.zeros((self.n,1)) 
		I = np.zeros((self.n,self.n)) 
		
		if measures: 

			z = np.zeros((self.m,1))
			R = np.zeros((self.m,self.m))

			for measure in measures:
				z[0,0] = measure.point.point.x
				z[1,0] = measure.point.point.y
				z[2,0] = measure.point.point.z

				R[0,0] = measure.point.covariance[0]
				R[0,1] = measure.point.covariance[1]
				R[0,2] = measure.point.covariance[2]
				R[1,0] = measure.point.covariance[3]
				R[1,1] = measure.point.covariance[4]
				R[1,2] = measure.point.covariance[5]
				R[2,0] = measure.point.covariance[6]
				R[2,1] = measure.point.covariance[7]
				R[2,2] = measure.point.covariance[8]

				i = i + self.H.T * R * z  
				I = I + self.H.T * R * self.H

		y = y_ + i
		Y = Y_ + I

		self.P = la.inv(Y)
		self.x = self.P  * y 

		theta = np.arctan2(self.x[3,0],self.x[1,0])
		q = tf.transformations.quaternion_from_euler(0, 0, theta)

		self.state.pose.position.x = self.x[0,0]
		self.state.pose.position.y = self.x[2,0]	
		self.state.pose.orientation.x = q[0]
		self.state.pose.orientation.y = q[1]
		self.state.pose.orientation.z = q[2]
		self.state.pose.orientation.w = q[3]

	def reset(self, reset):
		self.x = np.matrix([reset.x, 0.0, reset.y, 0.0]).T	

