import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Float32MultiArray
import cv2
from pointcloud import generate_pointcloud
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
import std_msgs.msg
from pca import compute_pca, compute_centroid

bridge = CvBridge()
bin_img = np.zeros((480,640,3)).astype('uint8')
point_cloud = []
pointcloud_publisher = rospy.Publisher("/pointcloud", PointCloud)
pose_publisher = rospy.Publisher("/current_pose", Float32MultiArray)
# pcl = PointCloud()
save = False

def depth_callback(data):
    global bin_img, point_cloud, pointcloud_publisher, save
    depth = np.reshape(np.asarray(data.data), (480,640))
    actual_depth = np.zeros((480,640))
    for i in range(480):
        actual_depth[i,:] = depth[479-i, :]
    point_cloud = generate_pointcloud(bin_img, actual_depth)
    if(not save):
        with open('pcl.txt', 'w') as f:
            for item in point_cloud:
                print >> f, item[0], item[1], item[2]
        if(len(point_cloud)>100):
            save = True
            print("Saved!!")
    if(len(point_cloud)>100):
        pose = compute_pca(np.asarray(point_cloud))
        centroid = compute_centroid(np.asarray(point_cloud))
        tmp = centroid[0]
        centroid[0] = centroid[1]
        centroid[1] = tmp
        msg = Float32MultiArray()
        pose_6d = [centroid[0], centroid[1], centroid[2], pose[0], pose[1], pose[2]]
        msg.data = pose_6d
        pose_publisher.publish(msg)
        # print(centroid)
    pcl = PointCloud()
    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'map'
    pcl.header = header
    for i in range(len(point_cloud)):
        pcl.points.append(Point32(point_cloud[i][0], point_cloud[i][1], point_cloud[i][2]))
    pointcloud_publisher.publish(pcl)
    # print("updated")

def rgb_callback(rgb_img):
    global bin_img
    cv_image = bridge.imgmsg_to_cv2(rgb_img, desired_encoding='passthrough')
    mask = ((cv_image[:,:,0]>100) & (cv_image[:,:,1]<50) & (cv_image[:,:,2]<50)).astype('uint8')*255
    bin_img[:,:,0] = mask
    bin_img[:,:,1] = mask
    bin_img[:,:,2] = mask
    kernel = cv2.getStructuringElement(cv2.MORPH_CROSS,(3,3))
    erode = cv2.erode(bin_img,kernel)
    bin_img = cv2.morphologyEx(erode,cv2.MORPH_OPEN,kernel)
    cv2.imwrite('tmp.png', bin_img)
    # cv2.waitKey(0)
    # print(mask)

def segment():
    global pointcloud_publisher
    rospy.init_node('segmentor_node', anonymous=True)
    rospy.Subscriber("/coppeliaSim/image/rgb", Image, rgb_callback)
    rospy.Subscriber("/coppeliaSim/image/depthd", Float32MultiArray, depth_callback)
    rospy.spin()

if __name__ == '__main__':
    print("zyo")
    segment()