import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray

o_M_jn = np.zeros([6,4,4])
o_M_e = np.zeros([4,4])
o_M_jn_e = np.zeros([7,4,4])
# vel_ee = np.asarray([0,-0.1,0,0,0,0])
e_J_e = np.zeros((6,6))
pub = rospy.Publisher("/jointRates_lj/coppeliaSim", Float32MultiArray)

def Inverse_T(T):

    R = T[:3,:3]
    t = T[:3,3]
    MAPPING_R1 = np.concatenate((np.transpose(R), np.matmul(-np.transpose(R),t).reshape([3,1])),axis=1)
    MAPPING_R2 = np.concatenate((np.zeros([1,3]), [[1]]),axis=1)
    MAPPING = np.concatenate((MAPPING_R1,MAPPING_R2),axis=0)
    Inverse_T = MAPPING
    return Inverse_T

def Manipulator_FK_Jacobian(T):
    Cross = np.zeros([3,1])
    H = np.round(T,4)
    J = np.zeros([6,6])
    for i in range(6):
        O_e = H[6,:3,3]
        O_i = H[i,:3,3] 
        Z_i = H[i,:3,2]

        O_p = O_e - O_i

        Cross[0,0] = Z_i[1]*O_p[2]-Z_i[2]*O_p[1]
        Cross[1,0] = Z_i[2]*O_p[0]-Z_i[0]*O_p[2]
        Cross[2,0] = Z_i[0]*O_p[1]-Z_i[1]*O_p[0]

        J[:3,i] = Cross[:,0]
        J[3:,i] = Z_i

    return J

def Jacobian_T(T):  # o_J_e  = Jacobian_o_T_e * e_J_e
    R = T[:3,:3]
    MAPPING_R1 = np.append(R,               np.zeros([3,3]), axis=1)
    MAPPING_R2 = np.append(np.zeros([3,3]), R,               axis=1)
    MAPPING = np.append(MAPPING_R1,MAPPING_R2,axis=0)       
    Jacobian_T = MAPPING
    return Jacobian_T 

def bTlj_callback(msg):
    global o_M_jn
    o_M_jn = np.reshape(np.asarray(msg.data),((6,4,4)))

def bTljee_callback(msg):
    global o_M_e, o_M_jn, e_J_e
    o_M_e = np.reshape(np.asarray(msg.data),(1,4,4))
    o_M_jn_e = np.append(o_M_jn, o_M_e, axis = 0)
    J = Manipulator_FK_Jacobian(o_M_jn_e)
    o_J_e = np.round(J,3)
    # print(o_J_e)
    condnumJ = np.linalg.cond(o_J_e,'fro')
    print(condnumJ)

    e_M_o = Inverse_T(o_M_jn_e[-1,:,:])
    e_J_o = Jacobian_T(e_M_o)
    e_J_e = np.matmul(e_J_o, o_J_e)

    # joint_vel = np.matmul(np.linalg.pinv(e_J_e), vel_ee)
    # msg = Float32MultiArray()
    # vel = [joint_vel[0], joint_vel[1], joint_vel[2], joint_vel[3], joint_vel[4], joint_vel[5]]
    # msg.data = vel
    # pub.publish(msg)
    # print(joint_vel)

def velocity_callback(msg):
    # print(np.asarray(msg.data))
    CurrentPointCloud_V_endeffvel = np.asarray(msg.data)
    Vrep_V_endeffvel = np.zeros(6)
    Vrep_V_endeffvel[0] = CurrentPointCloud_V_endeffvel[0]
    Vrep_V_endeffvel[1] = CurrentPointCloud_V_endeffvel[2]
    Vrep_V_endeffvel[2] = -1*CurrentPointCloud_V_endeffvel[1]
    Vrep_V_endeffvel[3] = CurrentPointCloud_V_endeffvel[0]
    Vrep_V_endeffvel[4] = CurrentPointCloud_V_endeffvel[5]
    Vrep_V_endeffvel[5] = -1*CurrentPointCloud_V_endeffvel[4]
    Vrep_V_endeffvel = 0.1*Vrep_V_endeffvel
    # print(Vrep_V_endeffvel)
    joint_vel = np.matmul(np.linalg.pinv(e_J_e), Vrep_V_endeffvel)
    msg = Float32MultiArray()
    vel = [joint_vel[0], joint_vel[1], joint_vel[2], joint_vel[3], joint_vel[4], joint_vel[5]]
    msg.data = vel
    pub.publish(msg)    


if __name__ == '__main__':
    rospy.init_node('jacobian_calculator', anonymous=True)
    rospy.Subscriber("/coppeliaSim/baseTransformation_lj", Float32MultiArray, bTlj_callback)
    rospy.Subscriber("/coppeliaSim/baseTransformation_llEndEffector", Float32MultiArray, bTljee_callback)
    rospy.Subscriber("/CurrentPointcloud_V_endeffectorvelocity", Float32MultiArray, velocity_callback)
    rospy.spin()