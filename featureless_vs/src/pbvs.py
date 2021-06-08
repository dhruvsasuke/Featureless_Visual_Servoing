import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from scipy.spatial.transform import Rotation as rr

ee_vel_publisher = rospy.Publisher("/CurrentPointcloud_V_endeffectorvelocity", Float32MultiArray)

desired_centroid = np.asarray((0.01, -0.02, 0.3))
desired_x = np.asarray((1,-1,0))
desired_y = np.asarray((-1,-1,0))
desired_z = np.asarray((0,0,-1))

desired_x = desired_x/np.linalg.norm(desired_x)
desired_y = desired_y/np.linalg.norm(desired_y)
desired_z = desired_z/np.linalg.norm(desired_z)

uV_camera_x = np.asarray([1,0,0])
uV_camera_y = np.asarray([0,1,0])
uV_camera_z = np.asarray([0,0,1])

dM11, dM12, dM13 = np.dot(uV_camera_x, desired_x), np.dot(uV_camera_x, desired_y), np.dot(uV_camera_x, desired_z)
dM21, dM22, dM23 = np.dot(uV_camera_y, desired_x), np.dot(uV_camera_y, desired_y), np.dot(uV_camera_y, desired_z)
dM31, dM32, dM33 = np.dot(uV_camera_z, desired_x), np.dot(uV_camera_z, desired_y), np.dot(uV_camera_z, desired_z)
camera_rM_desired = np.asarray([[dM11, dM12, dM13], 
                                [dM21, dM22, dM23], 
                                [dM31, dM32, dM33]])

def pose_callback(current_pose):
    global camera_rM_desired, desired_centroid
    current_centroid = np.asarray((current_pose.data[0], current_pose.data[1], current_pose.data[2]))
    current_x = np.asarray((current_pose.data[3], current_pose.data[4], current_pose.data[5]))
    current_y = np.asarray((current_pose.data[6], current_pose.data[7], current_pose.data[8]))
    current_z = np.cross(current_x, current_y)/np.linalg.norm(np.cross(current_x, current_y))
    
    M11, M12, M13 = np.dot(uV_camera_x, current_x), np.dot(uV_camera_x, current_y), np.dot(uV_camera_x, current_z)
    M21, M22, M23 = np.dot(uV_camera_y, current_x), np.dot(uV_camera_y, current_y), np.dot(uV_camera_y, current_z)
    M31, M32, M33 = np.dot(uV_camera_z, current_x), np.dot(uV_camera_z, current_y), np.dot(uV_camera_z, current_z)
    camera_rM_current = np.asarray([[M11, M12, M13], 
                                    [M21, M22, M23], 
                                    [M31, M32, M33]])
    current_rM_camera = np.transpose(camera_rM_current)
    current_rM_desired = np.matmul(current_rM_camera, camera_rM_desired)
    Rot = rr.from_dcm(current_rM_desired)
    current_V_axis = Rot.as_rotvec()
    camera_V_axis = np.matmul(camera_rM_current, current_V_axis)
    # print(np.matmul(camera_rM_current, current_V_axis), np.linalg.norm(np.matmul(camera_rM_current, current_V_axis)))
    camera_V_translation = (desired_centroid - current_centroid)
    current_V_translation = np.matmul(current_rM_camera, camera_V_translation)
    # print(current_x, current_y, current_z)
    msg = Float32MultiArray()
    if(np.linalg.norm(camera_V_translation)<0.017):
        current_V_translation = np.zeros(3)
    if(np.linalg.norm(camera_V_axis)<0.1):
        current_V_axis = np.zeros(3)
    msg.data = (current_V_translation[0], current_V_translation[1], current_V_translation[2], current_V_axis[0], current_V_axis[1], current_V_axis[2])
    ee_vel_publisher.publish(msg)
    # print(camera_V_translation, camera_V_axis)
    print(np.linalg.norm(camera_V_translation), np.linalg.norm(camera_V_axis))
    

if __name__ == '__main__':
    print("Visual Servoing started")
    rospy.init_node('servoing_node', anonymous=True)
    rospy.Subscriber("/current_pose", Float32MultiArray, pose_callback)
    rospy.spin()