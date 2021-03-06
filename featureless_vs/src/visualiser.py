from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
import pptk

def get_pts(infile):
    data = np.loadtxt('pcl.txt', delimiter=' ')
    return data #returns X,Y,Z points skipping the first 12 lines
	
def plot_ply(data):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(data[:,0], data[:,1], data[:,2], c='r', marker='o')
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
    plt.show()	
	
if __name__ == '__main__':
    print("yyyy")
    data = np.loadtxt('pcl.txt')
    print(data.shape)
    v = pptk.viewer(data)
    # print(data.shape)
