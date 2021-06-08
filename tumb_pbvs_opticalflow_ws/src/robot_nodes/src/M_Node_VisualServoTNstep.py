#!/usr/bin/env python2.7
from math import *
import numpy as np 
import cv2
from cv_bridge import CvBridge, CvBridgeError 

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
    '''
    Viusal Servoing
    '''
########################################################################################
#### -- Initialize Node --
    rospy.init_node('VisualServoing')  
########################################################################################
#### -- Subscribing --
#### --Image--

    pub_simStart=rospy.Publisher('/startSimulation', Bool,queue_size=10)
    pub_simPause=rospy.Publisher('/pauseSimulation', Bool,queue_size=10)
    pub_simStop=rospy.Publisher('/stopSimulation', Bool,queue_size=10)
    pub_simenableSynMode=rospy.Publisher('/enableSyncMode', Bool,queue_size=10)
    pub_simtriggerNextStep=rospy.Publisher('/triggerNextStep', Bool,queue_size=10)

    cb_simStepDone=channel.Subscribe('/simulationStepDone',Bool,(1))
    cb_simState=channel.Subscribe('/simulationState',Int32,(1))
    cb_simTime=channel.Subscribe('/simulationTime',Float32,(1))
    
    cb_desired_current = channel.Subscribe('/coppeliaSim/cd_M_c',Float32MultiArray,(4,4))

    cb_cameraCurrent_uVO1 = channel.Subscribe('/coppeliaSim/cameraCurrentO1',Float32MultiArray,(3))
    cb_cameraDesired_uVO2 = channel.Subscribe('/coppeliaSim/cameraDesiredO2',Float32MultiArray,(3))

    cb_bbox = channel.Subscribe('/bounding_box',Int32MultiArray,(4))
    cb_bbox1 = channel.Subscribe('/bounding_box1',Int32MultiArray,(4))

    cb_rgbFrame = channel.Subscribe('/coppeliaSim/image/rgb',Image,(480,640,3))
    cb_depthFrame = channel.Subscribe('/coppeliaSim/image/depth',Float32MultiArray,(480,640))
    cb_rgbFramed = channel.Subscribe('/coppeliaSim/image/rgbd',Image,(480,640,3))
    cb_depthFramed = channel.Subscribe('/coppeliaSim/image/depthd',Float32MultiArray,(480,640))

    cb_base_hM_camera = channel.Subscribe('/coppeliaSim/baseTTransformation_rj',Float32MultiArray,(4,4))

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
        # jointState_position_cb[i] = vs.Subscribe('/coppeliaSim/jointState_' + jtree[i] ,JointState,(6,1))

        pub_jointRates[i] = rospy.Publisher('/jointRates_'+ jtree[i]+'/coppeliaSim',Float32MultiArray,queue_size=10)

    # cb_base_hM_camera = channel.Subscribe('/coppeliaSim/baseTransformation_rj',Float32MultiArray,(4,4))

    pub_poseEstimated = rospy.Publisher('/PoseEstimation/coppeliaSim',Float32MultiArray,queue_size=10)

    pub_qA = rospy.Publisher('/qA/coppeliaSim',Float32MultiArray,queue_size=10)
    pub_qB = rospy.Publisher('/qB/coppeliaSim',Float32MultiArray,queue_size=10)

    cb_A = channel.Subscribe('/coppeliaSim/A',Float32MultiArray,(3))
    cb_B = channel.Subscribe('/coppeliaSim/B',Float32MultiArray,(3))
    
    ###############################################################################################################
    #### -- Publishing --
    rospy.sleep(2)

    i_FrameMean = []
    i_FrameMean1 = []
    xyz_cam = np.zeros([2,3])

    # bbox = np.load('/home/dhruv/dhBBPBVS.npy')
    # bbox = list(bbox)
    # bbox1 = np.load('/home/dhruv/dhBBPBVSd.npy')
    # bbox1 = list(bbox1)

    # while len(i_FrameMean) <30:
    # # for j in range(30):
    #     cb_depthFrame.S_data_callback(cb_depthFrame.msg.data)
    #     depthFrame = cb_depthFrame.data

    #     # cb_bbox.S_data_callback(cb_bbox.msg.data)
    #     # bbox = cb_bbox.data

    #     # r_i,r_n,c_i,c_n = (480/144)*bbox[2],(480/144)*bbox[3],(640/192)*bbox[0],(640/192)*bbox[1]
    #     r_i,r_n,c_i,c_n = bbox[0],bbox[1],bbox[2],bbox[3]
        
    #     boundBox = depthFrame[r_i:r_n,c_i:c_n]
    #     boundBox = boundBox[boundBox<1]
    #     i_FrameMean.append(np.mean(boundBox))

    #     # boundBox_Cen = depthFrame[(r_i+r_n)/2,(c_i+c_n)/2]
    #     # print(boundBox_Cen)
    #     # if boundBox_Cen < 1:
    #     #     i_FrameMean.append(boundBox_Cen)
    #     #     N_FrameMean = np.mean(i_FrameMean)
        
    #     N_FrameMean = np.mean(i_FrameMean)

    # while len(i_FrameMean1) <30:
    #     cb_depthFramed.S_data_callback(cb_depthFramed.msg.data)
    #     depthFramed = cb_depthFramed.data

    #     # cb_bbox1.S_data_callback(cb_bbox1.msg.data)
    #     # bbox1 = cb_bbox1.data

    #     # r_i1,r_n1,c_i1,c_n1 = (480/144)*bbox1[2],(480/144)*bbox1[3],(640/192)*bbox1[0],(640/192)*bbox1[1]
    #     r_i1,r_n1,c_i1,c_n1 = bbox1[0],bbox1[1],bbox1[2],bbox1[3]
        
    #     boundBox1 = depthFramed[r_i1:r_n1,c_i1:c_n1]
    #     boundBox1 = boundBox1[boundBox1<1]
    #     i_FrameMean1.append(np.mean(boundBox1))

    #     # boundBox_Cen1 = depthFramed[(r_i1+r_n1)/2,(c_i1+c_n1)/2]
    #     # if boundBox_Cen1 < 1:
    #     #     i_FrameMean1.append(boundBox_Cen1)
    #     #     N_FrameMean1 = np.mean(i_FrameMean1)


    #     N_FrameMean1 = np.mean(i_FrameMean1)
        

    # Z = N_FrameMean

    # u = (c_i + c_n)/2
    # v = (r_i + r_n)/2
    # u_ = u - 320
    # v_ = v - 240

    # fl = 530.0

    # X = (u_/fl)*Z
    # Y = (v_/fl)*Z   

    # Z1 = N_FrameMean1

    # u1 = (c_i1 + c_n1)/2
    # v1 = (r_i1 + r_n1)/2
    # u_1 = u1 - 320
    # v_1 = v1 - 240

    # fl1 = 530.0

    # X1 = (u_1/fl1)*Z1
    # Y1 = (v_1/fl1)*Z1   

    # # xyz_cam[0] = [X,Y,Z]  
    # # xyz_cam[1] = [X1,Y1,Z1] 
    # cb_A.S_data_callback(cb_A.msg.data) 
    # xyz_cam[0] = cb_A.data
    # cb_B.S_data_callback(cb_B.msg.data) 
    # xyz_cam[1] = cb_B.data


    # ################################################################################################################

    # rospy.sleep(2)
    # bridge = CvBridge()
    # simenableSynMode = channel.Publish(pub_simenableSynMode,Bool)
    # simtriggerNextStep = channel.Publish(pub_simtriggerNextStep,Bool)
    # simStart = channel.Publish(pub_simStart,Bool)
    # simPause = channel.Publish(pub_simPause,Bool)

    

    # simenableSynMode._msg.data = True
    # simenableSynMode.P_data(simenableSynMode._msg)

    while not rospy.is_shutdown():

    #     key = cv2.waitKey(1) & 0xFF

    #     simStepDone = cb_simStepDone.msg.data

    #     if simStepDone == True:
    #         simtriggerNextStep._msg.data = True
    #         simtriggerNextStep.P_data(simtriggerNextStep._msg)

        ###########################################################################################################
        #### --Image--
        #### --Kinematics--

        for i in range(1):
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

        cb_A.S_data_callback(cb_A.msg.data) 
        xyz_cam[0] = cb_A.data
        cb_B.S_data_callback(cb_B.msg.data) 
        xyz_cam[1] = cb_B.data

        #######################################
        cb_rgbFrame.S_data_callback(bridge.imgmsg_to_cv2(cb_rgbFrame.msg, 'bgr8'))
        rgbFrame = cb_rgbFrame.data

        cb_depthFrame.S_data_callback(cb_depthFrame.msg.data)
        depthFrame = cb_depthFrame.data
        #######################################
        cb_rgbFramed.S_data_callback(bridge.imgmsg_to_cv2(cb_rgbFramed.msg, 'bgr8'))
        rgbFramed = cb_rgbFramed.data

        cb_depthFramed.S_data_callback(cb_depthFramed.msg.data)
        depthFramed = cb_depthFramed.data
        ################################################################################################


        cb_base_hM_camera.S_data_callback(cb_base_hM_camera.msg.data)
        base_hM_camera = cb_base_hM_camera.data


        ########################################################################################################################################
        
        stepP = [0,0.2]

        dataPoint = np.zeros([len(stepP),3])
        bataPoint = np.zeros([len(stepP),3])

        # current
        ########################################################################################################################################
        #### @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        # r_i,r_n,c_i,c_n = bbox[2],bbox[3],bbox[0],bbox[1]

        camera_V_pointOnAxisOfRotation = xyz_cam[0]
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
  
        camera_V_pointOnAxisOfRotation = xyz_cam[1]
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

        ############################################################################################################################################
        # $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$


        stepP = np.zeros(len(VqA))
        dataPoint = np.zeros([len(stepP),3])
        bataPoint = np.zeros([len(stepP),3])
        dataPoint = VqA
        bataPoint = VqB

##############################################################  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5  ##########################################
        k = 0.01
        print(np.linalg.norm(cameraDesired_V_camera_O + cameraCurrent_V_O_camera), np.linalg.norm(cameraDesired_uVO2 - cameraCurrent_uVO1), condnumJ)
        cameraNext_V_camera_O = -cameraCurrent_V_O_camera + k*(cameraDesired_V_camera_O + cameraCurrent_V_O_camera)
        cameraNext_uVO2 = cameraCurrent_uVO1 + k*(cameraDesired_uVO2 - cameraCurrent_uVO1)
        cameraNext_uVO2 = cameraNext_uVO2/np.linalg.norm(cameraNext_uVO2)

        mag = np.dot(cameraNext_V_camera_O, cameraNext_uVO2)

        cameraCurrent_V_centre = -cameraCurrent_V_O_camera - mag*cameraCurrent_uVO1
        cameraCurrent_uV_centre = cameraCurrent_V_centre/np.linalg.norm(cameraCurrent_V_centre)
        # radius = (0.2)*np.linalg.norm(np.cross(cameraNext_V_camera_O/np.linalg.norm(cameraNext_V_camera_O), cameraNext_uVO2))
        radius = np.linalg.norm(cameraNext_V_camera_O - mag*cameraNext_uVO2)

        cameraCurrent_V_radius = np.cross(cameraCurrent_uVO1, cameraCurrent_uV_centre)
        cameraCurrent_V_radius = radius*cameraCurrent_V_radius/np.linalg.norm(cameraCurrent_V_radius)
        
        # rad_tmp = cameraCurrent_V_radius
        error = 1000
        theta = 0
        for i in range(360):
            rt = rr.from_rotvec(i*(np.pi/180)*cameraCurrent_uVO1)
            rad_tmp = rt.apply(cameraCurrent_V_radius)
            circ_temp = cameraCurrent_V_centre + rad_tmp
            if(np.linalg.norm(circ_temp)<error):
                error = np.linalg.norm(circ_temp)
                theta = i

        rt = rr.from_rotvec(theta*(np.pi/180)*cameraCurrent_uVO1)
        cameraCurrent_V_radius = rt.apply(cameraCurrent_V_radius)        
        cameraCurrent_V_circumference = cameraCurrent_V_centre + cameraCurrent_V_radius

        cameraCurrent_uV_axis = np.cross(cameraCurrent_uVO1, cameraNext_uVO2)/np.linalg.norm(np.cross(cameraCurrent_uVO1, cameraNext_uVO2))
        angle = acos(np.dot(cameraCurrent_uVO1, cameraNext_uVO2))
        rot = rr.from_rotvec(-angle*cameraCurrent_uV_axis)
        cameraCurrent_R_cameraTemp = rot.as_dcm()
        cameraCurrent_V_translation = cameraCurrent_V_circumference
        cameraCurrent_T_cameraTemp = np.r_[np.c_[np.asarray(cameraCurrent_R_cameraTemp), np.reshape(cameraCurrent_V_translation, (3,1))], [[0,0,0,1]] ]

        angle2 = 0
        error = 1000

        for i in range(360):
            temp_rot = rr.from_rotvec(i*(np.pi/180)*cameraNext_uVO2)
            cameraTemp_R_cameraNext = temp_rot.as_dcm()
            cameraTemp_T_cameraNext = np.r_[np.c_[np.asarray(cameraTemp_R_cameraNext), [[0],[0],[0]]], [[0,0,0,1]] ]
            cameraCurrent_T_cameraNext = np.matmul(cameraCurrent_T_cameraTemp , cameraTemp_T_cameraNext)
            cameraNext_V_pointing = np.matmul(ks.Kinematics(cameraCurrent_T_cameraNext).Inverse_T(), np.append(-cameraCurrent_V_O_camera, [1]))
            if(acos(np.dot(cameraNext_V_camera_O/np.linalg.norm(cameraNext_V_camera_O), cameraNext_V_pointing[:-1]/np.linalg.norm(cameraNext_V_pointing[:-1]))) < error):
                error = acos(np.dot(cameraNext_V_camera_O/np.linalg.norm(cameraNext_V_camera_O), cameraNext_V_pointing[:-1]/np.linalg.norm(cameraNext_V_pointing[:-1])))
                angle2 = i

        temp1_rot = rr.from_rotvec(angle2*(np.pi/180)*cameraNext_uVO2)
        cameraTemp_R_cameraNext  = temp1_rot.as_dcm()
        cameraTemp_T_cameraNext = np.r_[np.c_[np.asarray(cameraTemp_R_cameraNext), [[0],[0],[0]]], [[0,0,0,1]] ]
        cameraCurrent_T_cameraNext = np.matmul(cameraCurrent_T_cameraTemp , cameraTemp_T_cameraNext)
        cameraNext_V_pointing = np.matmul(ks.Kinematics(cameraCurrent_T_cameraNext).Inverse_T(), np.append(-cameraCurrent_V_O_camera, [1]))

        cameraNext_T_cameraCurrent = ks.Kinematics(cameraCurrent_T_cameraNext).Inverse_T()
        cameraNext_R_cameraCurrent = cameraNext_T_cameraCurrent[:3,:3]
        cameraCurrent_R_cameraNext = ks.Kinematics(cameraNext_T_cameraCurrent).Inverse_T()[:3,:3]

        time = 0.1
        rt = rr.from_dcm(cameraCurrent_R_cameraNext)
        rt = rt.as_rotvec()
        

        # print(rt)

        # print('pointing', cameraDesired_V_pointing, cameraDesired_V_camera_O)
        # print('error', np.linalg.norm(cameraDesired_V_camera_O - cameraDesired_V_pointing[:-1]))
        # print("pred_aor", np.matmul(cameraDesired_T_cameraCurrent[:3,:3], cameraCurrent_uVO1))
        # print("aor", cameraDesired_uVO2)
        # print(cameraCurrent_uVO1, cameraDesired_uVO2)

##############################################################  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5  #######################################

        poseEstimated = channel.Publish(pub_poseEstimated,Float32MultiArray)
        poseEstimated._msg.data = cameraCurrent_T_cameraNext.flatten()
        poseEstimated.P_data(poseEstimated._msg)

        
        #test
        cartVelocity = np.zeros([6,1])
        # cartVelocity[0] = -0.1
        cartVelocity[0] = cameraCurrent_V_circumference[0]/time
        cartVelocity[1] = cameraCurrent_V_circumference[1]/time
        cartVelocity[2] = cameraCurrent_V_circumference[2]/time
        cartVelocity[3] = rt[0]/time
        cartVelocity[4] = rt[1]/time
        cartVelocity[5] = rt[2]/time
        # print(cartVelocity)
        jointVelocity = np.matmul(np.linalg.pinv(e_J_e[0]),cartVelocity)
        for i in range(1):
            # print("pubss")
            jointRates = channel.Publish(pub_jointRates[i],Float32MultiArray)
            jointRates._msg.data = jointVelocity.flatten()
            jointRates.P_data(jointRates._msg)

        ############ UPDATE VARIABLES #################    
    
    simenableSynMode._msg.data = False
    simenableSynMode.P_data(simenableSynMode._msg)


########################################################################################

if __name__ =='__main__':
    print(main.__doc__)
    
    main()
    



























