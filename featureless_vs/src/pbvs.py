import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray


def pose_callback(current_pose):
    pass

if __name__ == '__main__':
    print("Visual Servoing started")
    rospy.init_node('servoing_node', anonymous=True)
    rospy.Subscriber("/current_pose", Float32MultiArray, pose_callback)