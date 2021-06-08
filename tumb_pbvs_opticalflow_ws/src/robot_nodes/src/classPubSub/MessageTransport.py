#!/usr/bin/env python2.7

from math import *
import numpy as np 
import rospy 

class Publish(object):
    def __init__(self,pub,msg_type):
        self._pub = pub
        self._msg = msg_type()
    def P_data(self,msg):
        self._msg = msg
        self._pub.publish(self._msg)

        
class Subscribe(object):
    def __init__(self,topicName,msgType,size):
        print("Class initiated")
        self.size = size
        self.data = np.zeros(self.size)
        self.msg = 0
        rospy.Subscriber(topicName,msgType,self.S_msg_callback)  
    def S_msg_callback(self,msg):
        print("CALLBACK!!!")
        self.msg = msg
        print(self.msg)
    def S_data_callback(self,msg_parameter):
        self.data = np.reshape(np.asarray(msg_parameter),self.size,order='C')
