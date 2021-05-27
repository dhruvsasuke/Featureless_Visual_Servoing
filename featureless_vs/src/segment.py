import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from pointcloud import generate_pointcloud

bridge = CvBridge()
bin_img = np.zeros((480,640,3)).astype('uint8')

def depth_callback(data):
    global bin_img
    depth_image = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
    point_cloud = generate_pointcloud(bin_img, depth_image)

def rgb_callback(rgb_img):
    global bin_img
    cv_image = bridge.imgmsg_to_cv2(rgb_img, desired_encoding='passthrough')
    mask = ((cv_image[:,:,0]>150) & (cv_image[:,:,1]<50) & (cv_image[:,:,2]<50)).astype('uint8')*255
    bin_img[:,:,0] = mask
    bin_img[:,:,1] = mask
    bin_img[:,:,2] = mask
    kernel = cv2.getStructuringElement(cv2.MORPH_CROSS,(3,3))
    erode = cv2.erode(bin_img,kernel)
    bin_img = cv2.morphologyEx(erode,cv2.MORPH_OPEN,kernel)
    # cv2.imshow('tmp', bin_img)
    # cv2.waitKey(0)
    # print(mask)
    

def segment():
    rospy.init_node('segmentor_node', anonymous=True)
    rospy.Subscriber("/coppeliaSim/image/rgb", Image, rgb_callback)
    rospy.Subscriber("/coppeliaSim/image/rgbd", Image, depth_callback)
    rospy.spin()

if __name__ == '__main__':
    print("zyo")
    segment()