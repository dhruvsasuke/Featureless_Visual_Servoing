from sklearn.decomposition import PCA
import numpy as np

def compute_pca(data):
    pca = PCA(n_components=2)
    pca.fit(data)
    # print("Vectors", pca.components_[0])
    # print(pca.components_.shape)
    return pca.components_
    # print("Variance", pca.explained_variance_ratio_)
    # return((pca.components_)[0])

def compute_centroid(data):
    x = np.sum(data[:,0])/(data[:,0].size)
    y = np.sum(data[:,1])/(data[:,1].size)
    z = np.sum(data[:,2])/(data[:,2].size)
    return(np.asarray((x,y,z)))