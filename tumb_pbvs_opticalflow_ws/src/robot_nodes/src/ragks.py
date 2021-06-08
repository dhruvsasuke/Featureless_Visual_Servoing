#!/usr/bin/env python2.7
from math import *
import numpy as np 
import cv2
from cv_bridge import CvBridge, CvBridgeError 
import rospy

# import sys
# sys.path.append('/home/dhruv/tumb_pbvs_opticalflow_ws/src/robot_nodes/src/classPubSub/MessageTransport.py')
import rospy 
from scipy.spatial.transform import Rotation as rr
#### Class Import
from classPubSub import MessageTransport as channel
#### Message Import
from Kinematics import K_Kinematics as ks
# import Kinematics.K_Kinematics as ks
from sensor_msgs.msg import Image,JointState
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Int32,Float32,Bool

def main():
    n = [6]
    jtree = ['rj']
    eetree =['rj']

    o_M_jn = np.zeros([1,6,4,4])
    o_M_e = np.zeros([1,4,4])
    o_M_jn_e = np.zeros([1,7,4,4])
    jointState_position = np.zeros([1,6,1])

    o_M_j_cb = [object]
    o_M_e_cb = [object]

    # jointState_position_cb = [object,object,object]
    pub_jointRates = [object]

    for i in range(len(n)):
        o_M_j_cb[i] = channel.Subscribe('/coppeliaSim/baseTransformation_'+jtree[i],Float32MultiArray,(n[i],4,4)) 
        o_M_e_cb[i] = channel.Subscribe('/coppeliaSim/baseTransformation_'+eetree[i] +'EndEffector',Float32MultiArray,(4,4))

        # print(o_M_j_cb[i].msg)
        # jointState_position_cb[i] = vs.Subscribe('/coppeliaSim/jointState_' + jtree[i] ,JointState,(6,1))

        pub_jointRates[i] = rospy.Publisher('/jointRates_'+ jtree[i]+'/coppeliaSim',Float32MultiArray,queue_size=10)

    while not rospy.is_shutdown():

        if(o_M_j_cb[i].msg == 0):
            continue

        for i in range(1):
            # if(o_M_j_cb[i].msg == 0):
            #     break
            o_M_j_cb[i].S_data_callback(o_M_j_cb[i].msg.data)
            o_M_e_cb[i].S_data_callback(o_M_e_cb[i].msg.data)
            o_M_jn[i] = o_M_j_cb[i].data
            o_M_e[i] = o_M_e_cb[i].data
            o_M_jn_e[i] = np.append(o_M_jn[i],[o_M_e[i]],axis = 0)
            # jointState_position_cb[i].S_data_callback(jointState_position_cb[i].msg.position)
            # jointState_position[i] = jointState_position_cb[i].data

        o_M_e_cb[-1].S_data_callback(o_M_e_cb[-1].msg.data)
        o_M_c = o_M_e_cb[-1].data

            #############################################################################################################
        o_J_e = np.zeros([2,6,6])
        e_J_e = np.zeros([2,6,6])
        for i in range(1):
            fk_jacobian_cb = ks.Kinematics(o_M_jn_e[i])
            fk_jacobian = fk_jacobian_cb.Manipulator_FK_Jacobian()
            o_J_e[i] = np.round(fk_jacobian,3)
            e_M_o = ks.Kinematics(o_M_jn_e[i,-1,:,:]).Inverse_T()
            e_J_o_cb = ks.Kinematics(e_M_o)
            e_J_o = e_J_o_cb.Jacobian_T()
            e_J_e[i] = np.matmul(e_J_o,o_J_e[i])

            # print(o_J_e[1])
        condnumJ = np.linalg.cond(o_J_e[0],'fro')
        print(e_J_e)

if __name__ == '__main__':
    # rospy.init_node('test_jacobian_node', anonymous=True)
    # while not rospy.is_shutdown():
    main()
    # rospy.spin()