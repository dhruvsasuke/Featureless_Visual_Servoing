#!/usr/bin/env python2.7
from math import *
import numpy as np 
import cv2
from cv2 import aruco
from cv_bridge import CvBridge, CvBridgeError 

import rospy 
from scipy.spatial.transform import Rotation as rr
#### Class Import
import classPubSub.MessageTransport as channel
# import Visual_Servoing.VS_VisualServoing as vs
#### Message Import
import Kinematics.K_Kinematics as ks
from sensor_msgs.msg import Image,JointState
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32MultiArray


def main():
    '''
    Viusal Servoing
    '''
########################################################################################
#### -- Initialize Node --
    rospy.init_node('VisualServoing')  
########################################################################################
#### -- Subscribing --
#### --Image--


    cb_desired_current = channel.Subscribe('/coppeliaSim/c_M_O1',Float32MultiArray,(4,4))

    cb_cameraCurrent_uVO1 = channel.Subscribe('/coppeliaSim/cameraCurrentO1',Float32MultiArray,(3))
    cb_cameraDesired_uVO2 = channel.Subscribe('/coppeliaSim/cameraDesiredO2',Float32MultiArray,(3))

    cb_bbox = channel.Subscribe('/bounding_box',Int32MultiArray,(4))

    cb_rgbFrame = channel.Subscribe('/coppeliaSim/image/rgb',Image,(480,640,3))
    cb_depthFrame = channel.Subscribe('/coppeliaSim/image/depth',Float32MultiArray,(480,640))
    cb_rgbFramed = channel.Subscribe('/coppeliaSim/image/rgbd',Image,(480,640,3))
    cb_depthFramed = channel.Subscribe('/coppeliaSim/image/depthd',Float32MultiArray,(480,640))


    cb_base_hM_camera = channel.Subscribe('/coppeliaSim/baseTransformation_rj',Float32MultiArray,(4,4))

    pub_poseEstimated = rospy.Publisher('/PoseEstimation/coppeliaSim',Float32MultiArray,queue_size=10)

    pub_qA = rospy.Publisher('/qA/coppeliaSim',Float32MultiArray,queue_size=10)
    pub_qB = rospy.Publisher('/qB/coppeliaSim',Float32MultiArray,queue_size=10)


    cb_A = channel.Subscribe('/coppeliaSim/A',Float32MultiArray,(6,3))
    cb_B = channel.Subscribe('/coppeliaSim/B',Float32MultiArray,(6,3))
    # pub_current = rospy.Publisher('/current/coppeliaSim',Float32MultiArray,queue_size=10)
   
########################################################################################
#### -- Publishing --
    i_FrameMean = []

    rospy.sleep(2)
    bridge = CvBridge()
    while not rospy.is_shutdown():
        key = cv2.waitKey(1) & 0xFF
        ###########################################################################################################
        #### --Image--
        ################################################################################################

        cb_A.S_data_callback(cb_A.msg.data)
        VqA = cb_A.data

        cb_B.S_data_callback(cb_B.msg.data)
        VqB = cb_B.data

        cb_desired_current.S_data_callback(cb_desired_current.msg.data)
        desired_current = cb_desired_current.data

        cb_cameraCurrent_uVO1.S_data_callback(cb_cameraCurrent_uVO1.msg.data)
        cameraCurrent_uVO1 = cb_cameraCurrent_uVO1.data

        cb_cameraDesired_uVO2.S_data_callback(cb_cameraDesired_uVO2.msg.data)
        cameraDesired_uVO2 = cb_cameraDesired_uVO2.data

        #######################################
        cb_rgbFrame.S_data_callback(bridge.imgmsg_to_cv2(cb_rgbFrame.msg, 'bgr8'))
        rgbFrame = cb_rgbFrame.data

        cb_depthFrame.S_data_callback(cb_depthFrame.msg.data)
        depthFrame = cb_depthFrame.data
        ######################################
        cb_rgbFramed.S_data_callback(bridge.imgmsg_to_cv2(cb_rgbFramed.msg, 'bgr8'))
        rgbFramed = cb_rgbFramed.data

        cb_depthFramed.S_data_callback(cb_depthFramed.msg.data)
        depthFramed = cb_depthFramed.data
        ################################################################################################


        cb_base_hM_camera.S_data_callback(cb_base_hM_camera.msg.data)
        base_hM_camera = cb_base_hM_camera.data


        ########################################################################################################################################
        # stepP = [0.01,0.02,0.04,0.05,0.08,0.09,0.01,0.05,0.09]
        stepP = [0.1,0.2,0.4,0.5,0.8,0.9,-0.1,-0.5,-0.9]

        dataPoint = np.zeros([len(stepP),3])
        bataPoint = np.zeros([len(stepP),3])





        # current
        ########################################################################################################################################
        #### @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        # r_i,r_n,c_i,c_n = bbox[2],bbox[3],bbox[0],bbox[1]

        r_i,r_n,c_i,c_n = [164, 343, 331, 452]

        
        boundBox = depthFrame[r_i:r_n,c_i:c_n]
        boundBox = boundBox[boundBox<1]
        i_FrameMean.append(np.mean(boundBox))

        N_FrameMean = np.mean(i_FrameMean)

        Z = 0.23

        u = (c_i + c_n)/2
        v = (r_i + r_n)/2
        u_ = u - 320
        v_ = v - 240

        fl = 530.0

        X = (u_/fl)*Z
        Y = (v_/fl)*Z


        camera_V_pointOnAxisOfRotation = np.asarray([X,Y,Z])
        cameraCurrent_V_O_camera = - camera_V_pointOnAxisOfRotation
        camera_uV_pointOnAxisOfRotation = camera_V_pointOnAxisOfRotation/np.linalg.norm(camera_V_pointOnAxisOfRotation)
        pointOnAxisOfRotation_uV_camera = - camera_uV_pointOnAxisOfRotation

        uV_axisOfRot_Z = cameraCurrent_uVO1
        uV_axisOfRot_Y = np.cross(pointOnAxisOfRotation_uV_camera,uV_axisOfRot_Z)
        uV_axisOfRot_X = np.cross(uV_axisOfRot_Y,uV_axisOfRot_Z)

        a = camera_V_pointOnAxisOfRotation
        for i in range(len(stepP)):
            b = stepP[i]
            C = np.dot(b,uV_axisOfRot_Z)
            c = np.sum(np.append([a],[C],axis=0),axis=0)
            dataPoint[i,:3] = c 


        uV_camera_Z = np.asarray([0,0,1])
        uV_camera_Y = np.asarray([0,1,0])
        uV_camera_X = np.asarray([1,0,0])
        
        
        M11,M12,M13 = np.dot(uV_camera_X,uV_axisOfRot_X),np.dot(uV_camera_X,uV_axisOfRot_Y),np.dot(uV_camera_X,uV_axisOfRot_Z)
        M21,M22,M23 = np.dot(uV_camera_Y,uV_axisOfRot_X),np.dot(uV_camera_Y,uV_axisOfRot_Y),np.dot(uV_camera_Y,uV_axisOfRot_Z)
        M31,M32,M33 = np.dot(uV_camera_Z,uV_axisOfRot_X),np.dot(uV_camera_Z,uV_axisOfRot_Y),np.dot(uV_camera_Z,uV_axisOfRot_Z)

        camera_rM_axisOfRot = np.asarray([[M11,M12,M13],
                        [M21,M22,M23],
                        [M31,M32,M33]])
        camera_tV_axisOfRot = camera_V_pointOnAxisOfRotation

        camera_T_axisOfRot = np.append(np.append(camera_rM_axisOfRot,np.reshape(camera_tV_axisOfRot,(3,1)),axis=1),np.asarray([[0,0,0,1]]),axis=0)

        cameraCurrent_T_axisOfRot = camera_T_axisOfRot


        base_T_O = np.matmul(base_hM_camera,cameraCurrent_T_axisOfRot)
        
        # Desired
        ########################################################################################################################################
        #### @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        # r_i,r_n,c_i,c_n = bbox[2],bbox[3],bbox[0],bbox[1]

        r_i,r_n,c_i,c_n = [180, 312, 348, 493]

        
        boundBox = depthFramed[r_i:r_n,c_i:c_n]
        boundBox = boundBox[boundBox<1]
        i_FrameMean.append(np.mean(boundBox))

        N_FrameMean = np.mean(i_FrameMean)

        Z = N_FrameMean

        Z = 0.31

        u = (c_i + c_n)/2
        v = (r_i + r_n)/2
        u_ = u - 320
        v_ = v - 240

        fl = 530.0

        X = (u_/fl)*Z
        Y = (v_/fl)*Z


        camera_V_pointOnAxisOfRotation = np.asarray([X,Y,Z])
        # print(camera_V_pointOnAxisOfRotation)
        cameraDesired_V_camera_O = camera_V_pointOnAxisOfRotation
        camera_uV_pointOnAxisOfRotation = camera_V_pointOnAxisOfRotation/np.linalg.norm(camera_V_pointOnAxisOfRotation)
        pointOnAxisOfRotation_uV_camera = - camera_uV_pointOnAxisOfRotation

        uV_axisOfRot_Z = cameraDesired_uVO2
        uV_axisOfRot_Y = np.cross(pointOnAxisOfRotation_uV_camera,uV_axisOfRot_Z)
        uV_axisOfRot_X = np.cross(uV_axisOfRot_Y,uV_axisOfRot_Z)

        a = camera_V_pointOnAxisOfRotation
        for i in range(len(stepP)):
            b = stepP[i]
            C = np.dot(b,uV_axisOfRot_Z)
            c = np.sum(np.append([a],[C],axis=0),axis=0)
            bataPoint[i,:3] = c 


        uV_camera_Z = np.asarray([0,0,1])
        uV_camera_Y = np.asarray([0,1,0])
        uV_camera_X = np.asarray([1,0,0])
        
        
        M11,M12,M13 = np.dot(uV_camera_X,uV_axisOfRot_X),np.dot(uV_camera_X,uV_axisOfRot_Y),np.dot(uV_camera_X,uV_axisOfRot_Z)
        M21,M22,M23 = np.dot(uV_camera_Y,uV_axisOfRot_X),np.dot(uV_camera_Y,uV_axisOfRot_Y),np.dot(uV_camera_Y,uV_axisOfRot_Z)
        M31,M32,M33 = np.dot(uV_camera_Z,uV_axisOfRot_X),np.dot(uV_camera_Z,uV_axisOfRot_Y),np.dot(uV_camera_Z,uV_axisOfRot_Z)

        camera_rM_axisOfRot = np.asarray([[M11,M12,M13],
                        [M21,M22,M23],
                        [M31,M32,M33]])
        camera_tV_axisOfRot = camera_V_pointOnAxisOfRotation

        camera_T_axisOfRot = np.append(np.append(camera_rM_axisOfRot,np.reshape(camera_tV_axisOfRot,(3,1)),axis=1),np.asarray([[0,0,0,1]]),axis=0)
        
        cameraDesired_T_axisOfRot = camera_T_axisOfRot

        ###########################################################################################################################
        # $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$


        stepP = np.zeros(len(VqA))
        dataPoint = np.zeros([len(stepP),3])
        bataPoint = np.zeros([len(stepP),3])
        dataPoint = VqA
        bataPoint = VqB
#########################################################  %%%%%%%%%%%%%%%%%   ####################################################################
        column_V = np.zeros([3,4])
        i=0
        for i in range(3):
            for j in range(3):
                opV = np.append(np.reshape(bataPoint[:,i],(len(stepP),1)),np.reshape(dataPoint[:,j],(len(stepP),1)),axis=1)
                res = np.sum(np.prod(opV,axis=1))
                column_V[i,j] = res 
            column_V[i,3] = np.sum(bataPoint[:,i])
        
        ndataPoint = len(stepP)
        A = np.asarray([[np.sum(np.square(dataPoint[:,0])),np.sum(np.prod(dataPoint[:,:2],axis=1)),np.sum(np.prod(dataPoint[:,::2],axis=1)),np.sum(dataPoint[:,0])],
            [np.sum(np.prod(dataPoint[:,:2],axis=1)),np.sum(np.square(dataPoint[:,1])),np.sum(np.prod(dataPoint[:,1:],axis=1)),np.sum(dataPoint[:,1])],
            [np.sum(np.prod(dataPoint[:,::2],axis=1)),np.sum(np.prod(dataPoint[:,1:],axis=1)),np.sum(np.square(dataPoint[:,2])),np.sum(dataPoint[:,2])],
            [np.sum(dataPoint[:,0]),np.sum(dataPoint[:,1]),np.sum(dataPoint[:,2]),ndataPoint]])


        r1 = np.matmul(np.linalg.pinv(A),np.transpose(column_V[0,:]))
        r2 = np.matmul(np.linalg.pinv(A),np.transpose(column_V[1,:]))
        r3 = np.matmul(np.linalg.pinv(A),np.transpose(column_V[2,:]))

        cdc = np.append(np.append(np.append([r1],[r2],axis=0),[r3],axis=0),np.asarray([[0,0,0,1]]),axis=0)

        VqA = dataPoint
        VqB = bataPoint
        
        dataPoint = np.zeros([len(stepP)+1,3])
        bataPoint = np.zeros([len(stepP)+1,3])

        dataPoint[:-1,:] = VqA
        dataPoint[-1,:] = [0,0,0]
        bataPoint[:-1,:] = VqB
        bataPoint[-1,:] = cdc[:3,3]

        # stepP = [0.01,0.02,0.04,0.05,0.08,0.09,0.01,0.05,0.09,0]
        stepP = np.zeros((len(VqA)+1))



        # dataPoint = np.asarray([[0.5449,0.1955,0.9227],
        # [0.6862,0.7202,0.8004],
        # [0.8936,0.7218,0.2859],
        # [0.0548,0.8778,0.5437],
        # [0.3037,0.5824,0.9848],
        # [0.0462,0.0707,0.7157]])

        # bataPoint = np.asarray([[2.5144,7.0691,1.9754],
        # [2.8292,7.4454,2.2224],
        # [3.3518,7.3060,2.1198],
        # [2.8392,7.8455,1.6229],
        # [2.4901,7.5449,1.9518],
        # [2.4273,7.1354,1.4349]])

        # stepP = np.zeros(6)

        column_V = np.zeros([3,4])
        i=0
        for i in range(3):
            for j in range(3):
                opV = np.append(np.reshape(bataPoint[:,i],(len(stepP),1)),np.reshape(dataPoint[:,j],(len(stepP),1)),axis=1)
                res = np.sum(np.prod(opV,axis=1))
                column_V[i,j] = res 
            column_V[i,3] = np.sum(bataPoint[:,i])
        
        ndataPoint = len(stepP)
        A = np.asarray([[np.sum(np.square(dataPoint[:,0])),np.sum(np.prod(dataPoint[:,:2],axis=1)),np.sum(np.prod(dataPoint[:,::2],axis=1)),np.sum(dataPoint[:,0])],
            [np.sum(np.prod(dataPoint[:,:2],axis=1)),np.sum(np.square(dataPoint[:,1])),np.sum(np.prod(dataPoint[:,1:],axis=1)),np.sum(dataPoint[:,1])],
            [np.sum(np.prod(dataPoint[:,::2],axis=1)),np.sum(np.prod(dataPoint[:,1:],axis=1)),np.sum(np.square(dataPoint[:,2])),np.sum(dataPoint[:,2])],
            [np.sum(dataPoint[:,0]),np.sum(dataPoint[:,1]),np.sum(dataPoint[:,2]),ndataPoint]])


        r1 = np.matmul(np.linalg.pinv(A),np.transpose(column_V[0,:]))
        r2 = np.matmul(np.linalg.pinv(A),np.transpose(column_V[1,:]))
        r3 = np.matmul(np.linalg.pinv(A),np.transpose(column_V[2,:]))

        cdc = np.append(np.append(np.append([r1],[r2],axis=0),[r3],axis=0),np.asarray([[0,0,0,1]]),axis=0)








        # print(cdc)
        # cdc = np.round(cdc,3)
        vv = np.matmul(cdc,cameraCurrent_T_axisOfRot)[:3,3]
        vvv = np.sqrt(np.sum(np.square(vv)))
        print(vvv)

        cameraDesired_R_cameraCurrent = cdc
        print('est',cameraDesired_R_cameraCurrent)
        print('vrep',desired_current)

##############################################################  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5  #######################################
        # cameraDesired_V_camera_O =
        cameraCurrent_R_cameraDesired = ks.Kinematics(cameraDesired_R_cameraCurrent).Inverse_T()[:3,:3] 
        cameraCurrent_V_cameraDesired_O = np.matmul(cameraCurrent_R_cameraDesired,cameraDesired_V_camera_O)
        # cameraCurrent_V_O_cameraCurrent =
        # print(cameraCurrent_V_cameraDesired_O)
        cameraCurrent_V_cameraCurrent_cameraDesired = np.sum(np.asarray([cameraCurrent_V_cameraDesired_O,cameraCurrent_V_O_camera]),axis=0)

        # print(cameraCurrent_V_cameraCurrent_cameraDesired)

        cameraCurrent_T_cameraDesired = np.append(np.append(cameraCurrent_R_cameraDesired,np.reshape(cameraCurrent_V_cameraCurrent_cameraDesired,[3,1]),axis=1),np.asarray([[0,0,0,1]]),axis=0)   

        # $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
        ############################################################################################################################

        poseEstimated = channel.Publish(pub_poseEstimated,Float32MultiArray)
        poseEstimated._msg.data = cameraCurrent_T_cameraDesired.flatten()
        poseEstimated.P_data(poseEstimated._msg)

        # PqA = channel.Publish(pub_qA,Float32MultiArray)
        # PqA._msg.data = qA.flatten()
        # PqA.P_data(PqA._msg)

        # PqB = channel.Publish(pub_qB,Float32MultiArray)
        # PqB._msg.data = qB.flatten()
        # PqB.P_data(PqB._msg)




########################################################################################

if __name__ =='__main__':
    print(main.__doc__)
    main()





























        # rros = rr.from_rotvec(-((np.pi/180)*90) * uV_axisOfRot_Z)
        # rro = rros.as_dcm()
        # # rro = np.append(np.append(rros,np.zeros((3,1)),axis=1),np.array([[0,0,0,1]]),axis=0)
        # print('rot',rro)
        # # R_zo = np.asarray([[cos(AangB),-sin(AangB),0,0],
        # #                     [sin(AangB),cos(AangB),0,0],
        # #                     [0,0,1,0],
        # #                     [0,0,0,1]])

        # # baseo1 = np.matmul(base_hM_camera,cameraCurrent_T_axisOfRot)

        # uV_axisOfRot_Y = np.matmul(rro,uV_axisOfRot_Y)
        # uV_axisOfRot_X = np.matmul(rro,uV_axisOfRot_X)
        # uV_axisOfRot_Z = uV_axisOfRot_Z

        # # # nY = np.matmul(baseo1,nY)
        # # # nX = np.matmul(baseo1,nX)
        # # # nZ = uV_axisOfRot_Z


        # uV_camera_Z = np.asarray([0,0,1])
        # uV_camera_Y = np.asarray([0,1,0])
        # uV_camera_X = np.asarray([1,0,0])
        
        
        # M11,M12,M13 = np.dot(uV_camera_X,uV_axisOfRot_X),np.dot(uV_camera_X,uV_axisOfRot_Y),np.dot(uV_camera_X,uV_axisOfRot_Z)
        # M21,M22,M23 = np.dot(uV_camera_Y,uV_axisOfRot_X),np.dot(uV_camera_Y,uV_axisOfRot_Y),np.dot(uV_camera_Y,uV_axisOfRot_Z)
        # M31,M32,M33 = np.dot(uV_camera_Z,uV_axisOfRot_X),np.dot(uV_camera_Z,uV_axisOfRot_Y),np.dot(uV_camera_Z,uV_axisOfRot_Z)

        # camera_rM_axisOfRot = np.asarray([[M11,M12,M13],
        #                 [M21,M22,M23],
        #                 [M31,M32,M33]])
        # camera_tV_axisOfRot = camera_V_pointOnAxisOfRotation

        # camerad_T_O1 = np.append(np.append(camera_rM_axisOfRot,np.reshape(camera_tV_axisOfRot,(3,1)),axis=1),np.asarray([[0,0,0,1]]),axis=0)
        
        # # cameraDesired_T_axisOfRot = camera_T_axisOfRot

        # # print(baseo1)


        # O1base = ks.Kinematics(baseO1).Inverse_T()
        # cdW = np.matmul(camerad_T_O1,O1base)
        # Wcd = ks.Kinematics(cdW).Inverse_T()